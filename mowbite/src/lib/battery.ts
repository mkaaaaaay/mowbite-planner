import type {Sample} from '@/hooks/useSensorHistory';
import {apiBase} from './mowers';

// the battery over weeks, from what docker/battery.sh keeps: a line per charge and per stretch between charges
export interface Cycle {
  kind: 'charge' | 'run';
  start: number; // unix s
  end: number;
  minutes: number; // charge: how long it took, run: blade minutes
  from: number; // volts
  to: number;
}

export function parseBattery(text: string): Cycle[] {
  const out: Cycle[] = [];
  for (const line of text.split('\n')) {
    const [kind, start, end, minutes, from, to] = line.trim().split(' ');
    if ((kind !== 'charge' && kind !== 'run') || !to) continue;
    out.push({kind, start: +start, end: +end, minutes: +minutes, from: +from, to: +to});
  }
  return out;
}

// full charges only, a top up after a short drive says nothing about the battery
export const chargeSamples = (cycles: Cycle[]): Sample[] =>
  cycles.filter((c) => c.kind === 'charge' && c.minutes >= 20).map((c) => ({t: c.end * 1000, v: c.minutes}));

// volts lost per hour with the blade on, only stretches with a fair bit of mowing
export const drainSamples = (cycles: Cycle[]): Sample[] =>
  cycles.filter((c) => c.kind === 'run' && c.minutes >= 15 && c.from > c.to).map((c) => ({t: c.end * 1000, v: (c.from - c.to) / (c.minutes / 60)}));

// the mean of the last four weeks against the first four the data has, once there's enough for both
export function trend(samples: Sample[], now = Date.now()): {first: number; last: number} | null {
  if (samples.length < 6) return null;
  const week = 7 * 86400e3;
  const t0 = samples[0].t;
  if (now - t0 < 6 * week) return null;
  const mean = (xs: Sample[]) => (xs.length ? xs.reduce((a, s) => a + s.v, 0) / xs.length : NaN);
  const first = mean(samples.filter((s) => s.t < t0 + 4 * week));
  const last = mean(samples.filter((s) => s.t > now - 4 * week));
  return Number.isFinite(first) && Number.isFinite(last) ? {first, last} : null;
}

// null: not served, an older container or MowBite not running as one
export async function loadBattery(): Promise<Cycle[] | null> {
  try {
    const res = await fetch(`${apiBase()}/cgi-bin/battery`, {cache: 'no-store'});
    if (!res.ok || !res.headers.get('content-type')?.includes('text/plain')) return null;
    return parseBattery(await res.text());
  } catch {
    return null;
  }
}
