// what changed from base to mine, put on top of theirs: two devices that saved settings from the same old copy
// both keep their change. a level deeper for objects (colors, icons), so two different colors don't overwrite
// each other. what both changed takes mine
type Obj = Record<string, unknown>;

const isObj = (v: unknown): v is Obj => !!v && typeof v === 'object' && !Array.isArray(v);
const same = (a: unknown, b: unknown) => JSON.stringify(a) === JSON.stringify(b);

export function mergeChanges<T extends object>(theirs: T, base: T, mine: T, depth = 2): T {
  const out: Obj = {...(theirs as Obj)};
  const b = base as Obj;
  const m = mine as Obj;
  for (const k of new Set([...Object.keys(b), ...Object.keys(m)])) {
    if (same(b[k], m[k])) continue;
    if (depth > 1 && isObj(m[k]) && isObj(b[k]) && isObj(out[k])) out[k] = mergeChanges(out[k] as Obj, b[k], m[k], depth - 1);
    else if (m[k] === undefined) delete out[k];
    else out[k] = m[k];
  }
  return out as T;
}
