'use client';

import {onTopic} from '@/lib/mqttClient';
import {useSyncExternalStore} from 'react';
import {TOPIC} from '@/lib/openmower';

export interface Position {
  x: number;
  y: number;
  heading: number;
}

// position/json comes ~7x a second, robot_state only once, so the marker glides instead of jumping
let position: Position | null = null;
let started = false;
const listeners = new Set<() => void>();

function start() {
  if (started) return;
  started = true;
  onTopic(TOPIC.position, (payload) => {
    try {
      const p = JSON.parse(payload.toString());
      if (typeof p.x !== 'number' || typeof p.y !== 'number') return;
      position = {x: p.x, y: p.y, heading: p.heading ?? 0};
    } catch {
      return;
    }
    listeners.forEach((l) => l());
  });
}

function subscribe(listener: () => void) {
  start();
  listeners.add(listener);
  return () => listeners.delete(listener);
}

// null if the mower doesn't publish position/json (older setups), use robot_state's pose then
export function useMowerPosition(): Position | null {
  return useSyncExternalStore(
    subscribe,
    () => position,
    () => null,
  );
}
