'use client';

import {topicStore} from '@/lib/mqttClient';
import {useSyncExternalStore} from 'react';
import type {Point} from './useMowerMap';
import {TOPIC} from '@/lib/openmower';

// lines the mower draws on top of the map, while recording: the outline green, obstacles red and the
// one being recorded right now blue
export interface OverlayLine {
  points: Point[];
  color: string;
  closed: boolean;
}

const store = topicStore<OverlayLine[]>(TOPIC.mapOverlay, [], (payload) => {
  const m: {polygons?: {polygon?: {points?: Point[]}; color?: string; closed?: boolean}[]} = JSON.parse(payload.toString());
  return (m.polygons ?? []).map((p) => ({points: p.polygon?.points ?? [], color: p.color ?? 'blue', closed: !!p.closed}));
});

export function useMapOverlay(): OverlayLine[] {
  return useSyncExternalStore(store.subscribe, store.get, store.initial);
}
