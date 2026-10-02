// app settings (map colors, icons). shared through the mower when the app runs from the docker image
// (docker/settings.cgi), otherwise per device. localStorage is also the cache so they're there
// before the request comes back.

import {mergeChanges} from './mergeChanges';
import {apiBase} from './mowers';
import {isApp} from './native';

export const COLORS = [
  {key: 'mower', label: 'Mower', value: '#ff1fa3'},
  {key: 'dock', label: 'Docking station', value: '#2196f3'},
  {key: 'track', label: 'Track', value: '#ff1fa3'},
  {key: 'transit', label: 'Driving without blades', value: '#ff1fa3'},
  {key: 'mow', label: 'Mowing area', value: '#4caf50'},
  {key: 'unmowed', label: 'Mowing area, not mowed', value: '#ffb300'},
  {key: 'nav', label: 'Navigation area', value: '#29b6f6'},
  {key: 'obstacle', label: 'Obstacle', value: '#ef5350'},
  {key: 'draft', label: 'Draft', value: '#888888'},
  {key: 'selected', label: 'Selected area', value: '#ff1fa3'},
  {key: 'vertex', label: 'Outline points', value: '#2196f3'},
  {key: 'stripes', label: 'Mowing direction', value: '#ff1fa3'},
] as const;

export type ColorKey = (typeof COLORS)[number]['key'];

// another mower running MowBite, the app can switch to it
export interface OtherMower {
  id: string;
  name: string;
  host: string; // name or ip in the local network
  appPort?: number; // MowBite, 8082
  wsPort?: number; // OpenMower's mqtt websocket, 9001
  prefix?: string;
}

export interface Settings {
  colors?: Partial<Record<ColorKey, string>>;
  // sizes are factors, 1 = default
  icons?: {mower?: string; dock?: string; mowerSize?: number; dockSize?: number; mowerRealSize?: boolean};
  // map on the dashboard: only while driving (default) or always
  dashboard?: {map?: 'auto' | 'always'};
  // weather on the dashboard from open-meteo, off unless switched on (it sends the rough position there)
  weather?: boolean;
  // the strip of grass along the bottom, on unless switched off
  grass?: boolean;
  // leaves drifting down in autumn, on unless switched off
  leaves?: boolean;
  // name of the mower this app runs on, and the other ones to switch to
  thisName?: string;
  mowers?: OtherMower[];
  // own aerial imagery source, xyz tile url
  imagery?: {url?: string; attribution?: string};
}

const STORAGE_KEY = 'appSettings';
// the settings of the mower the app is looking at (in the android app that's always another one)
const settingsUrl = () => (isApp() ? apiBase() : '') + '/cgi-bin/settings';

const NONE: Settings = {};
let current: Settings | null = null;
let shared = false;
const listeners = new Set<() => void>();

function load(): Settings {
  if (current) return current;
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    // colors used to be stored on their own
    const old = localStorage.getItem('mapColors');
    current = raw ? JSON.parse(raw) : old ? {colors: JSON.parse(old)} : {};
  } catch {
    current = {};
  }
  return current ?? NONE;
}

export function applyColors(colors: Settings['colors'] = {}) {
  const root = document.documentElement.style;
  for (const c of COLORS) root.setProperty(`--c-${c.key}`, colors[c.key] ?? c.value);
}

function setLocal(next: Settings) {
  current = next;
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(next));
  } catch {}
  applyColors(next.colors);
  listeners.forEach((l) => l());
}

// for useSyncExternalStore, the prerendered page has no saved settings (serverSnapshot)
export const settingsStore = {
  subscribe(l: () => void) {
    listeners.add(l);
    return () => listeners.delete(l);
  },
  snapshot: () => load(),
  serverSnapshot: () => NONE,
};

export function sharedSettings() {
  return shared;
}

// the settings as the container has them, and its version of them (null from an older container). a save says
// which version it started from, when another device saved in between the container answers 409 with its
// settings and the change made here goes on top of those (docker/settings.cgi)
let server: Settings = {};
let version: string | null = null;

let synced = false;
export async function syncSettings() {
  if (synced) return;
  synced = true;
  try {
    const res = await fetch(settingsUrl(), {cache: 'no-store'});
    if (!res.ok || !res.headers.get('content-type')?.includes('json')) return;
    const data = await res.json();
    shared = true;
    server = data && typeof data === 'object' ? data : {};
    version = res.headers.get('X-Settings-Version') || null;
    setLocal(server);
  } catch {
    // not served by the container, keep it local
  }
}

async function push(next: Settings) {
  for (let tries = 0; tries < 3; tries++) {
    const res = await fetch(settingsUrl() + (version ? `?v=${encodeURIComponent(version)}` : ''), {
      method: 'POST',
      body: JSON.stringify(next),
    });
    const v = res.headers.get('X-Settings-Version') || null;
    if (res.status !== 409) {
      if (res.ok) [server, version] = [next, v];
      return;
    }
    const theirs: Settings = await res.json();
    next = mergeChanges(theirs, server, next);
    [server, version] = [theirs, v];
    setLocal(next);
  }
}

// sliders and color pickers fire on every move, only the last state goes to the mower. one save after another
let saveTimer: ReturnType<typeof setTimeout> | undefined;
let saving: Promise<void> = Promise.resolve();
export function saveSettings(next: Settings) {
  setLocal(next);
  if (!shared) return;
  clearTimeout(saveTimer);
  saveTimer = setTimeout(() => {
    saving = saving.then(() => push(current ?? next)).catch(() => {});
  }, 400);
}

// inline in <head>: sets the color variables before anything is drawn, so saved colors don't flash
export const COLOR_BOOT_SCRIPT = `try{var s=JSON.parse(localStorage.getItem('${STORAGE_KEY}')||'null'),c=(s&&s.colors)||JSON.parse(localStorage.getItem('mapColors')||'{}'),d=${JSON.stringify(
  Object.fromEntries(COLORS.map((c) => [c.key, c.value])),
)};for(var k in d)document.documentElement.style.setProperty('--c-'+k,c[k]||d[k])}catch(e){}`;
