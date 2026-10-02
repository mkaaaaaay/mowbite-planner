import {useSyncExternalStore} from 'react';

// unsaved changes in the map editor. they're kept while the app runs, so another tab and back loses
// nothing: the map tab shows a dot, and reloading, closing or switching mowers asks first
let unsaved = false;
const listeners = new Set<() => void>();

const ask = (e: BeforeUnloadEvent) => {
  e.preventDefault();
  // older browsers only ask with this set
  e.returnValue = '';
};

export function setUnsavedMap(v: boolean) {
  if (v === unsaved) return;
  unsaved = v;
  if (v) window.addEventListener('beforeunload', ask);
  else window.removeEventListener('beforeunload', ask);
  listeners.forEach((l) => l());
}

function subscribe(l: () => void) {
  listeners.add(l);
  return () => listeners.delete(l);
}

export function useUnsavedMap(): boolean {
  return useSyncExternalStore(
    subscribe,
    () => unsaved,
    () => false,
  );
}
