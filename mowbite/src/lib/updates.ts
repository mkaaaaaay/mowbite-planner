import {useEffect, useSyncExternalStore} from 'react';
import {apiBase} from './mowers';

// Looks on github for a newer release. Off by default (it tells github your ip), switched on per
// device in the settings, then once a day. "check now" works either way.

const REPO = 'mkaaaaaay/mowbite';
const ENABLED_KEY = 'updateCheck';
const INFO_KEY = 'updateInfo';
const DISMISSED_KEY = 'updateDismissed';
const DAY = 24 * 3600e3;

export const APP_VERSION = process.env.APP_VERSION ?? '0.0.0';

export interface Release {
  version: string;
  url: string; // release page
  apk?: string;
}

// -1 when a is older than b. a pre-release (1.1.0-dev.3) comes before its release (1.1.0)
export function compareVersions(a: string, b: string): number {
  const parse = (v: string) => {
    const [core, pre] = v.replace(/^v/, '').split('-', 2);
    return {nums: core.split('.').map((n) => Number(n) || 0), pre};
  };
  const x = parse(a);
  const y = parse(b);
  for (let i = 0; i < 3; i++) {
    const d = (x.nums[i] ?? 0) - (y.nums[i] ?? 0);
    if (d) return Math.sign(d);
  }
  if (x.pre && !y.pre) return -1;
  if (!x.pre && y.pre) return 1;
  if (x.pre && y.pre) return x.pre.localeCompare(y.pre, undefined, {numeric: true});
  return 0;
}

export const isNewer = (release: string, current: string) => compareVersions(release, current) > 0;

function read<T>(key: string): T | null {
  try {
    const v = localStorage.getItem(key);
    return v ? (JSON.parse(v) as T) : null;
  } catch {
    return null;
  }
}

function write(key: string, value: unknown) {
  try {
    if (value === null) localStorage.removeItem(key);
    else localStorage.setItem(key, JSON.stringify(value));
  } catch {}
}

interface State {
  enabled: boolean;
  latest: Release | null;
  checkedAt: number | null;
  checking: boolean;
  failed: boolean;
  dismissed: string | null;
  // the version the mower's container runs, when known
  mower: string | null;
}

const EMPTY: State = {enabled: false, latest: null, checkedAt: null, checking: false, failed: false, dismissed: null, mower: null};
let state: State | null = null;
const listeners = new Set<() => void>();

function current(): State {
  if (!state) {
    const info = read<{latest: Release; at: number}>(INFO_KEY);
    state = {
      ...EMPTY,
      enabled: read<boolean>(ENABLED_KEY) === true,
      latest: info?.latest ?? null,
      checkedAt: info?.at ?? null,
      dismissed: read<string>(DISMISSED_KEY),
    };
  }
  return state;
}

function set(patch: Partial<State>) {
  state = {...current(), ...patch};
  listeners.forEach((l) => l());
}

export async function checkNow(): Promise<void> {
  set({checking: true, failed: false});
  try {
    const res = await fetch(`https://api.github.com/repos/${REPO}/releases/latest`, {cache: 'no-store'});
    if (!res.ok) throw new Error(String(res.status));
    const r = (await res.json()) as {tag_name: string; html_url: string; assets?: {name: string; browser_download_url: string}[]};
    const latest: Release = {
      version: r.tag_name.replace(/^v/, ''),
      url: r.html_url,
      apk: r.assets?.find((a) => a.name.endsWith('.apk'))?.browser_download_url,
    };
    const at = Date.now();
    write(INFO_KEY, {latest, at});
    set({latest, checkedAt: at, checking: false});
  } catch {
    set({checking: false, failed: true});
  }
}

export function setUpdateCheck(on: boolean) {
  write(ENABLED_KEY, on || null);
  set({enabled: on});
  if (on) void checkNow();
}

export function dismissUpdate(version: string) {
  write(DISMISSED_KEY, version);
  set({dismissed: version});
}

async function loadMowerVersion() {
  try {
    const res = await fetch(`${apiBase()}/version.json`, {cache: 'no-store'});
    if (res.ok) set({mower: ((await res.json()) as {version?: string}).version ?? null});
  } catch {}
}

let started = false;

// the update state, starts the daily check (if switched on) and asks the mower for its version
export function useUpdates(): State {
  const s = useSyncExternalStore(
    (l) => {
      listeners.add(l);
      return () => listeners.delete(l);
    },
    current,
    () => EMPTY,
  );
  useEffect(() => {
    if (started) return;
    started = true;
    void loadMowerVersion();
    const c = current();
    if (c.enabled && (!c.checkedAt || Date.now() - c.checkedAt > DAY)) void checkNow();
  }, []);
  return s;
}
