'use client';

import {topicStore} from '@/lib/mqttClient';
import {callRpc} from '@/lib/rpc';
import {useSyncExternalStore} from 'react';
import {RPC, TOPIC} from '@/lib/openmower';
import {closedRings, openRings} from '@/lib/rings';

export interface Point {
  x: number;
  y: number;
}

export interface MapArea {
  id: string;
  properties: {
    name?: string;
    type?: string;
    active?: boolean;
    // drivable but left out when mowing, needs an openmower that knows it (older ones drop it when saving)
    // false: drivable but not mowed, like a navigation area
    mowable?: boolean;
    // with mowable false: the mowing areas it lies in plan around it instead of across, needs an openmower that knows it
    mow_around?: boolean;
    // per area overrides, missing = the mower's global setting
    outline_count?: number;
    outline_overlap_count?: number;
    outline_offset?: number;
    angle?: number; // rad
    // rad, the final angle (offset and increment included) stays within, bouncing back at the ends. needs an
    // openmower that knows it
    angle_min?: number;
    angle_max?: number;
  };
  outline: Point[];
}

export interface DockingStation {
  id: string;
  position: Point;
  heading: number;
}

export interface MowerMap {
  areas: MapArea[];
  docking_stations: DockingStation[];
}

// kept for the whole run, map/json is retained and comes only once
const store = topicStore<MowerMap | null>(TOPIC.map, null, (payload) => openRings(JSON.parse(payload.toString())));

export function useMowerMap(): MowerMap | null {
  return useSyncExternalStore(store.subscribe, store.get, store.initial);
}

// map.replace takes the same shape as map/json, outlines closed like the mower records them
export function saveMap(map: MowerMap): Promise<unknown> {
  return callRpc(RPC.replaceMap, [closedRings(map)]);
}
