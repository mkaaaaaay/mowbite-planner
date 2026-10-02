import type {Point} from '@/hooks/useMowerMap';

export function polygonArea(o: Point[]): number {
  let sum = 0;
  for (let i = 0; i < o.length; i++) {
    const a = o[i];
    const b = o[(i + 1) % o.length];
    sum += a.x * b.y - b.x * a.y;
  }
  return Math.abs(sum / 2);
}

export function containsPoint(o: Point[], x: number, y: number): boolean {
  let inside = false;
  for (let i = 0, j = o.length - 1; i < o.length; j = i++) {
    if (o[i].y > y !== o[j].y > y && x < ((o[j].x - o[i].x) * (y - o[i].y)) / (o[j].y - o[i].y) + o[i].x) {
      inside = !inside;
    }
  }
  return inside;
}

// how much of outline a lies inside outline b, by its points (0..1)
export function shareInside(a: Point[], b: Point[]): number {
  if (!a.length) return 0;
  return a.filter((p) => containsPoint(b, p.x, p.y)).length / a.length;
}
