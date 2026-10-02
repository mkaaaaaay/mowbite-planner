import type {Point} from '@/hooks/useMowerMap';
import type {TrackSegment} from '@/hooks/useMowHistory';
import {containsPoint} from './geometry';

// distance to the nearest outline edge and that edge's direction
function nearestEdge(p: Point, o: Point[]): {dist: number; dir: number} {
  let best = {dist: Infinity, dir: 0};
  for (let i = 0; i < o.length; i++) {
    const a = o[i];
    const b = o[(i + 1) % o.length];
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const len2 = dx * dx + dy * dy;
    const t = len2 ? Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2)) : 0;
    const dist = Math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
    if (dist < best.dist) best = {dist, dir: Math.atan2(dy, dx)};
  }
  return best;
}

// outline passes run along the edge, a run that's parallel to the nearest edge and this close to it
// is one of them, however many passes the mower was set to
const ALONG_EDGE = 1.5;

const angleOf = (a: [number, number], b: [number, number]) => Math.atan2(b[1] - a[1], b[0] - a[0]);

function angleDiff(a: number, b: number) {
  return Math.abs(((((a - b) % (2 * Math.PI)) + 3 * Math.PI) % (2 * Math.PI)) - Math.PI);
}

// Direction the stripes were actually mowed in (deg, 0..180) from a recorded track. Only straight runs
// with the blade on, inside the area and away from its edge (the outline passes run along the edge).
export function measuredStripeAngle(
  segments: TrackSegment[],
  outline: Point[],
  edgeMargin: number,
): {angleDeg: number; meters: number} | null {
  const bins = new Array(180).fill(0);
  let total = 0;

  for (const s of segments) {
    if (!s.attributes.blades) continue;
    const p = s.points;
    let i = 0;
    while (i < p.length - 1) {
      let j = i + 1;
      const dir = angleOf(p[i], p[i + 1]);
      while (j < p.length - 1 && angleDiff(angleOf(p[j], p[j + 1]), dir) < (6 * Math.PI) / 180) j++;
      const len = Math.hypot(p[j][0] - p[i][0], p[j][1] - p[i][1]);
      const mid = {x: (p[i][0] + p[j][0]) / 2, y: (p[i][1] + p[j][1]) / 2};
      const run = angleOf(p[i], p[j]);
      const edge = nearestEdge(mid, outline);
      const alongEdge = edge.dist <= ALONG_EDGE && Math.min(angleDiff(run, edge.dir), angleDiff(run, edge.dir + Math.PI)) < (6 * Math.PI) / 180;
      if (len >= 2 && containsPoint(outline, mid.x, mid.y) && edge.dist > edgeMargin && !alongEdge) {
        const deg = ((((run * 180) / Math.PI) % 180) + 180) % 180;
        bins[Math.floor(deg) % 180] += len;
        total += len;
      }
      i = j;
    }
  }

  if (total < 20) return null;
  // strongest 5 degree window
  let bestSum = 0;
  let bestAt = 0;
  for (let d = 0; d < 180; d++) {
    let sum = 0;
    for (let k = -2; k <= 2; k++) sum += bins[(d + k + 180) % 180];
    if (sum > bestSum) {
      bestSum = sum;
      bestAt = d;
    }
  }
  // not clearly one direction, e.g. only outline passes or a very short job
  if (bestSum < total * 0.4) return null;
  return {angleDeg: bestAt + 0.5, meters: bestSum};
}

// signed difference b - a between two stripe directions in degrees, stripes have no front so it's mod 180
export function stripeAngleDiff(a: number, b: number): number {
  return ((((b - a) % 180) + 270) % 180) - 90;
}
