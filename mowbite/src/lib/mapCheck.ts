import type {MapArea, MowerMap, Point} from '@/hooks/useMowerMap';
import {containsPoint, shareInside} from './geometry';

// Things in a map the mower can't work with well, shown in the editor and asked about before saving.
// warn: the mower may plan or drive wrong, hint: only has no effect
export type Problem =
  | {kind: 'crossing'; level: 'warn'; areaId: string; at: Point}
  | {kind: 'points'; level: 'warn'; areaId: string}
  // at: the approach point in front of the dock
  | {kind: 'dock'; level: 'warn'; dockId: string; at: Point; distance: number}
  | {kind: 'outside'; level: 'hint'; areaId: string};

const EPS = 1e-9;
const cross = (o: Point, a: Point, b: Point) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
const onSegment = (p: Point, a: Point, b: Point) =>
  Math.min(a.x, b.x) - EPS <= p.x && p.x <= Math.max(a.x, b.x) + EPS && Math.min(a.y, b.y) - EPS <= p.y && p.y <= Math.max(a.y, b.y) + EPS;

// where segments ab and cd meet, touching included. null if they don't
function meet(a: Point, b: Point, c: Point, d: Point): Point | null {
  const d1 = cross(c, d, a);
  const d2 = cross(c, d, b);
  const d3 = cross(a, b, c);
  const d4 = cross(a, b, d);
  if (((d1 > EPS && d2 < -EPS) || (d1 < -EPS && d2 > EPS)) && ((d3 > EPS && d4 < -EPS) || (d3 < -EPS && d4 > EPS))) {
    const t = d1 / (d1 - d2);
    return {x: a.x + t * (b.x - a.x), y: a.y + t * (b.y - a.y)};
  }
  if (Math.abs(d1) <= EPS && onSegment(a, c, d)) return a;
  if (Math.abs(d2) <= EPS && onSegment(b, c, d)) return b;
  if (Math.abs(d3) <= EPS && onSegment(c, a, b)) return c;
  if (Math.abs(d4) <= EPS && onSegment(d, a, b)) return d;
  return null;
}

// the first place where a closed outline crosses or touches itself, null if it doesn't. sorted by x and only
// compared while the x ranges overlap, so outlines recorded with many points stay quick
export function selfCrossing(o: Point[]): Point | null {
  const n = o.length;
  if (n < 4) return null;
  const segs = o.map((a, i) => {
    const b = o[(i + 1) % n];
    return {i, a, b, minX: Math.min(a.x, b.x), maxX: Math.max(a.x, b.x), minY: Math.min(a.y, b.y), maxY: Math.max(a.y, b.y)};
  });
  segs.sort((s, t) => s.minX - t.minX);
  for (let p = 0; p < n; p++) {
    const s = segs[p];
    for (let q = p + 1; q < n && segs[q].minX <= s.maxX + EPS; q++) {
      const t = segs[q];
      // neighbours share a point
      const gap = Math.abs(s.i - t.i);
      if (gap === 1 || gap === n - 1) continue;
      if (t.minY > s.maxY + EPS || t.maxY < s.minY - EPS) continue;
      const at = meet(s.a, s.b, t.a, t.b);
      if (at) return at;
    }
  }
  return null;
}

// outlines are kept per area object, an area the editor didn't change isn't checked again
const crossings = new WeakMap<Point[], Point | null>();
function crossingOf(o: Point[]): Point | null {
  if (!crossings.has(o)) crossings.set(o, selfCrossing(o));
  return crossings.get(o)!;
}

const usable = (a: MapArea) => a.properties.active !== false && (a.properties.type === 'mow' || a.properties.type === 'nav');

// approachDistance: docking_approach_distance of the mower (m), 1.5 is its default
export function checkMap(map: MowerMap, approachDistance = 1.5): Problem[] {
  const problems: Problem[] = [];
  // drafts are ignored by the mower
  const areas = map.areas.filter((a) => a.properties.type !== 'draft');
  const drivable = areas.filter((a) => usable(a) && a.outline.length > 2);
  for (const a of areas) {
    if (a.outline.length < 3) {
      problems.push({kind: 'points', level: 'warn', areaId: a.id});
      continue;
    }
    const at = crossingOf(a.outline);
    if (at) problems.push({kind: 'crossing', level: 'warn', areaId: a.id, at});
    if (a.properties.type === 'obstacle' && a.properties.active !== false && !drivable.some((d) => shareInside(a.outline, d.outline) > 0)) {
      problems.push({kind: 'outside', level: 'hint', areaId: a.id});
    }
  }
  // to dock the mower plans its way to a point this far in front of the dock and drives the rest straight in
  // without the map (DockingBehavior.cpp). the dock itself may lie outside, usually it does, that point may not
  for (const d of map.docking_stations ?? []) {
    const at = {
      x: d.position.x - Math.cos(d.heading) * approachDistance,
      y: d.position.y - Math.sin(d.heading) * approachDistance,
    };
    if (!drivable.some((a) => containsPoint(a.outline, at.x, at.y))) {
      problems.push({kind: 'dock', level: 'warn', dockId: d.id, at, distance: approachDistance});
    }
  }
  return problems;
}
