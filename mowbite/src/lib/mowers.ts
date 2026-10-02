import {isApp} from './native';
import {settingsStore, type OtherMower} from './settings';

// which mower this device looks at: '' is the one the app is served from. switching reloads the
// app, so every connection, cache and page starts clean on the other mower
const KEY = 'mower';
// in the app the list is kept on the phone, there is no mower of its own to keep it on
const APP_KEY = 'appMowers';

export function appMowers(): OtherMower[] {
  try {
    return JSON.parse(localStorage.getItem(APP_KEY) ?? '[]');
  } catch {
    return [];
  }
}

export function saveAppMowers(list: OtherMower[]) {
  try {
    localStorage.setItem(APP_KEY, JSON.stringify(list));
  } catch {}
}

// the mowers there are to switch to: the phone's list in the app, the shared one on a mower
export function mowerList(): OtherMower[] {
  return isApp() ? appMowers() : (settingsStore.snapshot().mowers ?? []);
}

export function selectedMowerId(): string {
  try {
    return localStorage.getItem(KEY) ?? '';
  } catch {
    return '';
  }
}

export function selectedMower(): OtherMower | null {
  const id = selectedMowerId();
  const list = mowerList();
  // the app always talks to one of its mowers, the first one if none is picked
  if (isApp()) return list.find((m) => m.id === id) ?? list[0] ?? null;
  return id ? (list.find((m) => m.id === id) ?? null) : null;
}

export function selectMower(id: string) {
  try {
    if (id) localStorage.setItem(KEY, id);
    else localStorage.removeItem(KEY);
  } catch {}
  window.location.reload();
}

// where the selected mower's MowBite answers (sensor history, backups, schedule)
export function apiBase(): string {
  if (typeof window === 'undefined') return '';
  const m = selectedMower();
  return m ? `http://${m.host}:${m.appPort ?? 8082}` : '';
}
