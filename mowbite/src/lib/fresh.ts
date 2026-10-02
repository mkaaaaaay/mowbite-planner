// Answers from the mower that were asked for moments ago are handed out again instead of asking a
// second time, e.g. when a page is rendered next to the current one while swiping and then opened.
const store = new Map<string, {at: number; p: Promise<unknown>}>();

export function fresh<T>(key: string, load: () => Promise<T>, maxAge = 15000): Promise<T> {
  const hit = store.get(key);
  if (hit && Date.now() - hit.at < maxAge) return hit.p as Promise<T>;
  const p = load();
  store.set(key, {at: Date.now(), p});
  // a failed answer isn't kept
  p.catch(() => {
    if (store.get(key)?.p === p) store.delete(key);
  });
  return p;
}

// after a change, so the next load asks again
export function forget(prefix: string) {
  for (const key of store.keys()) if (key.startsWith(prefix)) store.delete(key);
}
