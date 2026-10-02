import type {MowerMap, Point} from '@/hooks/useMowerMap';

// OpenMower records outlines closed, the last point repeats the first. Other apps rely on that
// (turf.js refuses a ring that isn't closed). In the editor that point would be a second handle on
// top of the first, so it's dropped on the way in and added again on the way out.

const same = (a: Point, b: Point) => Math.abs(a.x - b.x) < 1e-9 && Math.abs(a.y - b.y) < 1e-9;

export function openRing(o: Point[]): Point[] {
  return o.length > 1 && same(o[0], o[o.length - 1]) ? o.slice(0, -1) : o;
}

export function closeRing(o: Point[]): Point[] {
  return o.length > 0 && !same(o[0], o[o.length - 1]) ? [...o, o[0]] : o;
}

const eachOutline = (m: MowerMap, f: (o: Point[]) => Point[]): MowerMap => ({
  ...m,
  areas: (m.areas ?? []).map((a) => ({...a, outline: f(a.outline ?? [])})),
});

// for the editor
export const openRings = (m: MowerMap) => eachOutline(m, openRing);
// for the mower, backups and files
export const closedRings = (m: MowerMap) => eachOutline(m, closeRing);
