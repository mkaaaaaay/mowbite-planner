'use client';

import {onTopic} from '@/lib/mqttClient';
import {useSyncExternalStore} from 'react';
import {PARAM_DEFAULTS, TOPIC} from '@/lib/openmower';

export type MowerParams = Record<string, unknown>;

const EMPTY: MowerParams = {};
let params: MowerParams = EMPTY;
let started = false;
const listeners = new Set<() => void>();

function start() {
  if (started) return;
  started = true;
  onTopic(TOPIC.params, (payload) => {
    try {
      params = JSON.parse(payload.toString());
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

// ros params of the mower, keys like "/mower_logic/tool_width"
export function useMowerParams(): MowerParams {
  return useSyncExternalStore(
    subscribe,
    () => params,
    () => EMPTY,
  );
}

// a number param, or mower_logic's default when the mower's list came without it (see PARAM_DEFAULTS). nothing
// before the list arrived, a default shown then could be wrong
export function numParam(params: MowerParams, key: string): number | undefined {
  const v = params[key];
  if (typeof v === 'number') return v;
  return params === EMPTY ? undefined : PARAM_DEFAULTS[key];
}

// gps datum, v2 keeps it under /ll/services/gps/, v1 under the gps driver, so match by the end of the key
export function datumFromParams(params: MowerParams): {lat: number; lon: number} | undefined {
  let lat: number | undefined;
  let lon: number | undefined;
  for (const [k, v] of Object.entries(params)) {
    if (typeof v !== 'number') continue;
    if (k.endsWith('/datum_lat')) lat = v;
    if (k.endsWith('/datum_long') || k.endsWith('/datum_lon')) lon = v;
  }
  return lat !== undefined && lon !== undefined && (lat !== 0 || lon !== 0) ? {lat, lon} : undefined;
}
