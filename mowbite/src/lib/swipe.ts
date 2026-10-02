import {useSyncExternalStore} from 'react';

// swiping between the pages is optional, per device (settings)
const KEY = 'swipePages';

export const swipeEnabled = () => {
  try {
    return localStorage.getItem(KEY) === '1';
  } catch {
    return false;
  }
};

const listeners = new Set<() => void>();

export const setSwipeEnabled = (on: boolean) => {
  try {
    if (on) localStorage.setItem(KEY, '1');
    else localStorage.removeItem(KEY);
  } catch {}
  listeners.forEach((l) => l());
};

export function useSwipeEnabled(): boolean {
  return useSyncExternalStore(
    (l) => {
      listeners.add(l);
      return () => listeners.delete(l);
    },
    swipeEnabled,
    () => false,
  );
}
