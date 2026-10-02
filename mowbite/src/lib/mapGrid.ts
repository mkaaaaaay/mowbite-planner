import type {Point} from '@/hooks/useMowerMap';

// meter grid lines for the visible part of the map, step picked so lines stay at least ~25px apart
export function meterGrid(
  view: {left: number; right: number; bottom: number; top: number},
  pxPerMeter: number,
): {step: number; xs: number[]; ys: number[]} {
  const step = [0.5, 1, 2, 5, 10, 20].find((s) => s * pxPerMeter >= 25) ?? 50;
  const range = (from: number, to: number) => {
    const out: number[] = [];
    for (let t = Math.ceil(from / step) * step; t <= to; t += step) out.push(t);
    return out;
  };
  return {step, xs: range(view.left, view.right), ys: range(view.bottom, view.top)};
}

// radius of the point handles: recorded outlines have tons of points, so they shrink when they're close together
export function handleRadius(outline: Point[], pxPerMeter: number): number {
  if (outline.length < 2) return 2.5;
  let perimeter = 0;
  for (let i = 0; i < outline.length; i++) {
    const prev = outline[(i + outline.length - 1) % outline.length];
    perimeter += Math.hypot(outline[i].x - prev.x, outline[i].y - prev.y);
  }
  return Math.min(2.5, Math.max(1.5, ((perimeter / outline.length) * pxPerMeter) / 3));
}
