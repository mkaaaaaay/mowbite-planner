import type {Point} from '@/hooks/useMowerMap';

function distToSegment(p: Point, a: Point, b: Point): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len2 = dx * dx + dy * dy;
  if (len2 === 0) return Math.hypot(p.x - a.x, p.y - a.y);
  const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
  return Math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

// douglas-peucker on an open line, keeps first and last point
function simplifyLine(points: Point[], tolerance: number): Point[] {
  if (points.length < 3) return points;

  let maxDist = 0;
  let index = 0;
  const first = points[0];
  const last = points[points.length - 1];
  for (let i = 1; i < points.length - 1; i++) {
    const d = distToSegment(points[i], first, last);
    if (d > maxDist) {
      maxDist = d;
      index = i;
    }
  }

  if (maxDist <= tolerance) return [first, last];
  const left = simplifyLine(points.slice(0, index + 1), tolerance);
  const right = simplifyLine(points.slice(index), tolerance);
  return [...left.slice(0, -1), ...right];
}

// tolerance in meters. the ring is cut at point 0 and the point farthest from it,
// both halves are simplified separately so the shape can't collapse
export function simplifyPolygon(outline: Point[], tolerance: number): Point[] {
  if (outline.length <= 4) return outline;

  let far = 0;
  let maxDist = 0;
  for (let i = 1; i < outline.length; i++) {
    const d = Math.hypot(outline[i].x - outline[0].x, outline[i].y - outline[0].y);
    if (d > maxDist) {
      maxDist = d;
      far = i;
    }
  }

  const a = simplifyLine(outline.slice(0, far + 1), tolerance);
  const b = simplifyLine([...outline.slice(far), outline[0]], tolerance);
  const result = [...a.slice(0, -1), ...b.slice(0, -1)];
  return result.length >= 3 ? result : outline;
}
