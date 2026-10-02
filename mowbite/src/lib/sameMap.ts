import type {MowerMap} from '@/hooks/useMowerMap';

// same content. areas the editor didn't touch keep their object, so only changed ones get compared as text
export function sameMap(a: MowerMap, b: MowerMap): boolean {
  if (a === b) return true;
  const same = (x: unknown, y: unknown) => x === y || JSON.stringify(x) === JSON.stringify(y);
  const rest = (m: MowerMap) => ({...m, areas: null, docking_stations: null});
  return (
    a.areas.length === b.areas.length &&
    a.areas.every((x, i) => same(x, b.areas[i])) &&
    same(a.docking_stations, b.docking_stations) &&
    same(rest(a), rest(b))
  );
}
