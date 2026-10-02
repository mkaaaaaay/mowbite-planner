import type {MowerMap} from '@/hooks/useMowerMap';

// The edits made here and a change made on the mower meanwhile (another app or device, a recording), put together
// by id: what only one side changed comes from that side, what both changed differently keeps the version from
// here and is reported. The order of the areas is the mowing order: whose order changed wins, new areas go in
// after the one they follow on their side.

type Item = {id: string};
const same = (a: unknown, b: unknown) => JSON.stringify(a) === JSON.stringify(b);

function mergeList<T extends Item>(base: T[], mine: T[], theirs: T[], conflicts: string[]): T[] {
  const byId = (list: T[]) => new Map(list.map((x) => [x.id, x]));
  const [b, m, t] = [byId(base), byId(mine), byId(theirs)];
  const pick = new Map<string, T>();
  for (const id of new Set([...b.keys(), ...m.keys(), ...t.keys()])) {
    const [bi, mi, ti] = [b.get(id), m.get(id), t.get(id)];
    let out: T | undefined;
    if (same(mi, bi)) out = ti;
    else if (same(ti, bi) || same(mi, ti)) out = mi;
    else {
      out = mi;
      conflicts.push(id);
    }
    if (out) pick.set(id, out);
  }
  const ids = (list: T[]) => list.map((x) => x.id);
  // the side that reordered leads, the other side's new areas are put in after their neighbour
  const theirsReordered = !same(ids(theirs).filter((id) => b.has(id)), ids(base).filter((id) => t.has(id)));
  const [lead, other] = theirsReordered ? [theirs, mine] : [mine, theirs];
  const order = ids(lead).filter((id) => pick.has(id));
  ids(other).forEach((id, i) => {
    if (!pick.has(id) || order.includes(id)) return;
    const before = ids(other)
      .slice(0, i)
      .reverse()
      .find((x) => order.includes(x));
    order.splice(before === undefined ? 0 : order.indexOf(before) + 1, 0, id);
  });
  return order.map((id) => pick.get(id)!);
}

export function mergeMaps(base: MowerMap, mine: MowerMap, theirs: MowerMap): {map: MowerMap; conflicts: string[]} {
  const conflicts: string[] = [];
  const map: MowerMap = {
    ...theirs,
    areas: mergeList(base.areas, mine.areas, theirs.areas, conflicts),
    docking_stations: mergeList(base.docking_stations ?? [], mine.docking_stations ?? [], theirs.docking_stations ?? [], conflicts),
  };
  return {map, conflicts};
}
