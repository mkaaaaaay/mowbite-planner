import * as ClipperLib from 'clipper-lib';
import type {Point} from '@/hooks/useMowerMap';
import {containsPoint, polygonArea} from './geometry';

// where the cut path crosses the outline: position along the path and along the outline (edge + t)
interface Crossing {
  pathPos: number;
  edge: number;
  t: number;
  point: Point;
}

function segmentHit(a: Point, b: Point, c: Point, d: Point): [number, number] | null {
  const rx = b.x - a.x;
  const ry = b.y - a.y;
  const sx = d.x - c.x;
  const sy = d.y - c.y;
  const den = rx * sy - ry * sx;
  if (Math.abs(den) < 1e-12) return null;
  const u = ((c.x - a.x) * sy - (c.y - a.y) * sx) / den;
  const v = ((c.x - a.x) * ry - (c.y - a.y) * rx) / den;
  return u >= 0 && u <= 1 && v >= 0 && v < 1 ? [u, v] : null;
}

// outline vertices strictly between two positions, walking forward around the ring
function walk(outline: Point[], from: Crossing, to: Crossing): Point[] {
  const n = outline.length;
  const out: Point[] = [];
  const sameEdgeAhead = from.edge === to.edge && to.t > from.t;
  if (sameEdgeAhead) return out;
  let k = (from.edge + 1) % n;
  for (let guard = 0; guard < n; guard++) {
    out.push(outline[k]);
    if (k === to.edge) break;
    k = (k + 1) % n;
  }
  return out;
}

// Splits the outline along a drawn path. Uses the first stretch of the path that runs through the
// area, from where it enters to where it leaves, so the path can bend and may start and end outside.
export function splitByPath(outline: Point[], path: Point[]): [Point[], Point[]] | null {
  if (outline.length < 3 || path.length < 2) return null;

  const hits: Crossing[] = [];
  for (let s = 0; s + 1 < path.length; s++) {
    for (let e = 0; e < outline.length; e++) {
      const h = segmentHit(path[s], path[s + 1], outline[e], outline[(e + 1) % outline.length]);
      if (!h) continue;
      const [u, t] = h;
      hits.push({
        pathPos: s + u,
        edge: e,
        t,
        point: {x: path[s].x + u * (path[s + 1].x - path[s].x), y: path[s].y + u * (path[s + 1].y - path[s].y)},
      });
    }
  }
  hits.sort((a, b) => a.pathPos - b.pathPos);

  const pointAt = (pos: number): Point => {
    const s = Math.min(Math.floor(pos), path.length - 2);
    const u = pos - s;
    return {x: path[s].x + u * (path[s + 1].x - path[s].x), y: path[s].y + u * (path[s + 1].y - path[s].y)};
  };

  for (let i = 0; i + 1 < hits.length; i++) {
    const a = hits[i];
    const b = hits[i + 1];
    if (b.pathPos - a.pathPos < 1e-9) continue;
    const mid = pointAt((a.pathPos + b.pathPos) / 2);
    if (!containsPoint(outline, mid.x, mid.y)) continue;

    // path corners between entry and exit
    const inner: Point[] = [];
    for (let k = Math.floor(a.pathPos) + 1; k <= Math.floor(b.pathPos) && k < path.length; k++) {
      if (k > a.pathPos && k < b.pathPos) inner.push(path[k]);
    }

    const ringA = [a.point, ...inner, b.point, ...walk(outline, b, a)];
    const ringB = [b.point, ...[...inner].reverse(), a.point, ...walk(outline, a, b)];
    if (ringA.length < 3 || ringB.length < 3 || polygonArea(ringA) < 0.01 || polygonArea(ringB) < 0.01) return null;
    return [ringA, ringB];
  }
  return null;
}

// clipper works in integers, this is 0.01 mm
const SCALE = 1e5;
const toPath = (o: Point[]) => o.map((p) => ({X: Math.round(p.x * SCALE), Y: Math.round(p.y * SCALE)}));
const fromPath = (p: ClipperLib.Path): Point[] => p.map((q) => ({x: q.X / SCALE, y: q.Y / SCALE}));

function clip(type: ClipperLib.ClipType, subject: ClipperLib.Paths, cut: ClipperLib.Paths): ClipperLib.Paths {
  const c = new ClipperLib.Clipper();
  c.AddPaths(subject, ClipperLib.PolyType.ptSubject, true);
  c.AddPaths(cut, ClipperLib.PolyType.ptClip, true);
  const out: ClipperLib.Paths = [];
  c.Execute(type, out, ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero);
  return out;
}

const perimeter = (o: Point[]) => o.reduce((sum, p, i) => sum + Math.hypot(o[(i + 1) % o.length].x - p.x, o[(i + 1) % o.length].y - p.y), 0);

// a point inside the shape: its centroid, or else the middle between two of its corners
function pointInside(shape: Point[]): Point | null {
  const c = {x: shape.reduce((s, p) => s + p.x, 0) / shape.length, y: shape.reduce((s, p) => s + p.y, 0) / shape.length};
  if (containsPoint(shape, c.x, c.y)) return c;
  for (let i = 0; i < shape.length; i++) {
    for (let j = i + 2; j < shape.length; j++) {
      const m = {x: (shape[i].x + shape[j].x) / 2, y: (shape[i].y + shape[j].y) / 2};
      if (containsPoint(shape, m.x, m.y)) return m;
    }
  }
  return null;
}

// Cuts a shape drawn inside the area out as an area of its own. An area can't have a hole, so the rest
// is cut in two by a straight line through the shape, turned so that this cut is as short as possible.
// Returns the two halves and the shape.
export function cutOut(outline: Point[], shape: Point[]): [Point[], Point[], Point[]] | null {
  if (outline.length < 3 || shape.length < 3) return null;
  if (!shape.every((p) => containsPoint(outline, p.x, p.y))) return null;
  for (let s = 0; s < shape.length; s++) {
    for (let e = 0; e < outline.length; e++) {
      if (segmentHit(shape[s], shape[(s + 1) % shape.length], outline[e], outline[(e + 1) % outline.length])) return null;
    }
  }
  // a shape that crosses itself isn't one area
  const clean = ClipperLib.Clipper.SimplifyPolygon(toPath(shape), ClipperLib.PolyFillType.pftNonZero);
  if (clean.length !== 1 || polygonArea(shape) < 0.01) return null;
  const inner = fromPath(clean[0]);
  const at = pointInside(inner);
  if (!at) return null;

  const rest = clip(ClipperLib.ClipType.ctDifference, [toPath(outline)], [toPath(inner)]);
  const far = 1000;
  let best: {halves: [Point[], Point[]]; cut: number} | null = null;
  for (let deg = 0; deg < 180; deg += 5) {
    const a = (deg * Math.PI) / 180;
    const d = {x: Math.cos(a) * far, y: Math.sin(a) * far};
    const side = (k: number): Point[] => [
      {x: at.x - d.x, y: at.y - d.y},
      {x: at.x + d.x, y: at.y + d.y},
      {x: at.x + d.x - k * d.y, y: at.y + d.y + k * d.x},
      {x: at.x - d.x - k * d.y, y: at.y - d.y + k * d.x},
    ];
    const one = clip(ClipperLib.ClipType.ctIntersection, rest, [toPath(side(1))]);
    const two = clip(ClipperLib.ClipType.ctIntersection, rest, [toPath(side(-1))]);
    // each side one piece without a hole, otherwise try another direction
    if (one.length !== 1 || two.length !== 1) continue;
    const halves: [Point[], Point[]] = [fromPath(one[0]), fromPath(two[0])];
    if (halves.some((h) => polygonArea(h) < 0.01)) continue;
    // both halves have the cut on their edge, the shape and the outline are the same every time
    const cut = perimeter(halves[0]) + perimeter(halves[1]);
    if (!best || cut < best.cut - 1e-6) best = {halves, cut};
  }
  return best && [best.halves[0], best.halves[1], inner];
}

export function generateId(): string {
  return Math.random().toString(36).slice(2) + Date.now().toString(36);
}
