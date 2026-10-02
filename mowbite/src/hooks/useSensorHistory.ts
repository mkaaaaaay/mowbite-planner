'use client';

import {onTopic} from '@/lib/mqttClient';
import {useSyncExternalStore} from 'react';
import {apiBase} from '@/lib/mowers';
import {TOPIC} from '@/lib/openmower';

export interface Sample {
  t: number; // ms
  v: number;
  // lowest and highest value in the recorder's 5 min bucket
  lo?: number;
  hi?: number;
}

const KEEP_MS = 60 * 60 * 1000;
const EVERY_MS = 5000;

// sensor id -> samples, oldest first. only covers the time the app has been open
let live: Record<string, Sample[]> = {};
// the last 24 h from the container's recorder (docker/recorder.sh), empty when there is none
let day: Record<string, Sample[]> = {};
// both together: recorder buckets up to where the live samples start
let history: Record<string, Sample[]> = {};

function merge() {
  const next: Record<string, Sample[]> = {};
  for (const id of new Set([...Object.keys(live), ...Object.keys(day)])) {
    const l = live[id] ?? [];
    const start = l.length ? l[0].t : Infinity;
    next[id] = [...(day[id] ?? []).filter((b) => b.t + 300000 <= start), ...l];
  }
  history = next;
  listeners.forEach((f) => f());
}

async function loadDay() {
  try {
    const res = await fetch(`${apiBase()}/cgi-bin/sensors`, {cache: 'no-store'});
    if (!res.ok || !res.headers.get('content-type')?.includes('text/plain')) return;
    const next: Record<string, Sample[]> = {};
    for (const line of (await res.text()).split('\n')) {
      const [id, t, lo, hi, v] = line.split('\t');
      if (!v) continue;
      (next[id] ??= []).push({t: +t * 1000, v: +v, lo: +lo, hi: +hi});
    }
    for (const list of Object.values(next)) list.sort((a, b) => a.t - b.t);
    day = next;
    merge();
  } catch {
    // not served by the container
  }
}
let started = false;
const listeners = new Set<() => void>();

export function startSensorHistory() {
  if (started) return;
  started = true;
  void loadDay();
  setInterval(loadDay, 5 * 60 * 1000);
  onTopic(TOPIC.sensorData, (payload, topic) => {
    const m = topic.match(/^sensors\/(.+)\/data$/);
    if (!m) return;
    const v = Number(payload.toString());
    if (!Number.isFinite(v)) return;
    const now = Date.now();
    const list = live[m[1]] ?? [];
    if (list.length && now - list[list.length - 1].t < EVERY_MS) return;
    const kept = list.length && now - list[0].t > KEEP_MS ? list.filter((s) => now - s.t <= KEEP_MS) : list;
    live = {...live, [m[1]]: [...kept, {t: now, v}]};
    merge();
  });
}

function subscribe(listener: () => void) {
  startSensorHistory();
  listeners.add(listener);
  return () => listeners.delete(listener);
}

const EMPTY: Record<string, Sample[]> = {};

export function useSensorHistory(): Record<string, Sample[]> {
  return useSyncExternalStore(
    subscribe,
    () => history,
    () => EMPTY,
  );
}
