import {useSyncExternalStore} from 'react';
import {THEME_KEY as KEY} from './themeBoot';

// light or dark per device, dark unless picked otherwise. 'auto' follows the device, the css reads data-theme on <html>.
// frost is dark with milky glass cards
export type ThemeChoice = 'auto' | 'light' | 'dark' | 'frost';

const listeners = new Set<() => void>();

function read(): ThemeChoice {
  try {
    const v = localStorage.getItem(KEY);
    if (v === 'light' || v === 'auto' || v === 'frost') return v;
  } catch {}
  return 'dark';
}

let choice: ThemeChoice | null = null;

export function setThemeChoice(c: ThemeChoice) {
  choice = c;
  try {
    localStorage.setItem(KEY, c);
  } catch {}
  if (c === 'auto') delete document.documentElement.dataset.theme;
  else document.documentElement.dataset.theme = c;
  listeners.forEach((l) => l());
}

export function useThemeChoice(): ThemeChoice {
  return useSyncExternalStore(
    (l) => {
      listeners.add(l);
      return () => listeners.delete(l);
    },
    () => (choice ??= read()),
    () => 'dark',
  );
}
