import type {Point} from '@/hooks/useMowerMap';
// the esm build only has a default export, despite what its types say
import polygonClipping, {type Ring} from 'polygon-clipping';

const toRing = (o: Point[]): Ring => o.map((p) => [p.x, p.y]);

// one outline for both areas, null if they don't touch. an area can't have holes in map.json,
// so a gap enclosed by the two gets filled in, holesFilled says if that happened
export function mergeOutlines(a: Point[], b: Point[]): {outline: Point[]; holesFilled: number} | null {
  const result = polygonClipping.union([toRing(a)], [toRing(b)]);
  if (result.length !== 1) return null;
  const [outer, ...holes] = result[0];
  // rings come back closed, first point repeated at the end
  const outline = outer.slice(0, -1).map(([x, y]) => ({x, y}));
  return {outline, holesFilled: holes.length};
}
