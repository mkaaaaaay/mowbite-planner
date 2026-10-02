import type {Point} from '@/hooks/useMowerMap';

// like the per area range in MowingBehavior.cpp. It's about the direction of the stripes, 0° and 180° give
// the same ones, so an angle whose stripes already lie in the range stays. Past an end it bounces back
export function angleInRange(angle: number, min?: number, max?: number): number {
  if (min === undefined || max === undefined) return angle;
  const half = Math.PI;
  const d = max - min;
  // half a turn or more allows every direction
  if (d >= half) return angle;
  // max < min is a range across the ends, e.g. 170° to -170° around 180°
  const width = ((d % half) + half) % half;
  if (width === 0) return min;
  // the same stripes can be written ±180° apart, use the one closest to the middle of the range
  const mid = min + width / 2;
  const r = (angle - mid) % half;
  angle = mid + (r > half / 2 ? r - half : r < -half / 2 ? r + half : r);
  let t = (angle - min) % (2 * width);
  if (t < 0) t += 2 * width;
  return min + (t <= width ? t : 2 * width - t);
}

// same as MowingBehavior.cpp: direction from the first outline point to the first one more than 2 m away
export function autoMowAngle(outline: Point[]): number {
  const first = outline[0];
  for (const p of outline) {
    const dx = p.x - first.x;
    const dy = p.y - first.y;
    if (Math.hypot(dx, dy) > 2) return Math.atan2(dy, dx);
  }
  return 0;
}
