'use client';

import {getMqttClient, onTopic} from '@/lib/mqttClient';
import {useSyncExternalStore} from 'react';
import {TOPIC} from '@/lib/openmower';

export interface MowerState {
  battery_percentage: number;
  gps_percentage: number;
  current_state: string;
  current_sub_state?: string;
  // while mowing: the mowing area (in map order), the path of its plan and the pose on that path, -1 otherwise
  current_area?: number;
  current_path?: number;
  current_path_index?: number;
  emergency: number;
  is_charging: number;
  rain_detected: number;
  pose: {
    x: number;
    y: number;
    heading: number;
    pos_accuracy: number;
  };
}

export interface MowerLink {
  // the last state that came, kept while the connection is gone
  state: MowerState | null;
  connected: boolean;
  // connected, but no state for a while: ros stopped, or the wifi is gone and the socket hasn't noticed yet
  stale: boolean;
  // ms, when the last state came
  lastAt: number;
}

// mower_logic sends its state once a second, this long without one means none is coming
const STALE_MS = 5000;

const NONE: MowerLink = {state: null, connected: false, stale: false, lastAt: 0};
// one store for every page, so remounting doesn't flash "waiting" or "connection lost"
let link = NONE;
let started = false;
const listeners = new Set<() => void>();

function update(patch: Partial<MowerLink>) {
  if ((Object.keys(patch) as (keyof MowerLink)[]).every((k) => link[k] === patch[k])) return;
  link = {...link, ...patch};
  listeners.forEach((l) => l());
}

function start() {
  if (started) return;
  started = true;
  const c = getMqttClient();
  c.on('connect', () => update({connected: true}));
  c.on('close', () => update({connected: false}));
  if (c.connected) update({connected: true});
  onTopic(TOPIC.robotState, (payload) => {
    try {
      update({state: JSON.parse(payload.toString()), lastAt: Date.now(), stale: false});
    } catch {
      // ignore malformed payloads
    }
  });
  setInterval(() => update({stale: !!link.state && Date.now() - link.lastAt > STALE_MS}), 1000);
}

function subscribe(listener: () => void) {
  start();
  listeners.add(listener);
  return () => listeners.delete(listener);
}

export function useMowerState(): MowerLink {
  return useSyncExternalStore(
    subscribe,
    () => link,
    () => NONE,
  );
}

// connected and the state is current: only then is what's shown real and a command goes out right away
export const isLive = (l: MowerLink) => l.connected && !l.stale;
