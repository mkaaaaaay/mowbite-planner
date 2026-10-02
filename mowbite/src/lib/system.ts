import {apiBase} from './mowers';

// memory, storage, load and temperature of the machine the container runs on (docker/system.cgi).
// remote: it doesn't run on the mower but on a pc or NAS, the numbers are that machine's then
export interface SystemInfo {
  remote: boolean;
  memTotal?: number; // bytes
  memAvailable?: number;
  diskTotal?: number; // the system disk
  diskFree?: number;
  dataTotal?: number; // the data volume, often the same disk
  dataFree?: number;
  load?: number; // 1 minute average
  cpus?: number;
  cpuTemp?: number; // °C
  uptime?: number; // s
}

export function parseSystem(text: string): SystemInfo {
  const v: Record<string, string[]> = {};
  for (const line of text.split('\n')) {
    const [key, ...rest] = line.trim().split(/\s+/);
    if (key) v[key] = rest;
  }
  const num = (key: string) => {
    const n = Number(v[key]?.[0]);
    return v[key] && Number.isFinite(n) ? n : undefined;
  };
  return {
    remote: v.remote?.[0] === '1',
    memTotal: num('mem_total'),
    memAvailable: num('mem_available'),
    diskTotal: num('disk_total'),
    diskFree: num('disk_free'),
    dataTotal: num('data_total'),
    dataFree: num('data_free'),
    load: num('load'),
    cpus: num('cpus'),
    cpuTemp: num('cpu_temp'),
    uptime: num('uptime'),
  };
}

// null: not served, an older container or MowBite not running as one
export async function loadSystem(): Promise<SystemInfo | null> {
  try {
    const res = await fetch(`${apiBase()}/cgi-bin/system`, {cache: 'no-store'});
    if (!res.ok || !res.headers.get('content-type')?.includes('text/plain')) return null;
    return parseSystem(await res.text());
  } catch {
    return null;
  }
}
