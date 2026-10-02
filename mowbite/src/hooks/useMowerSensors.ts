'use client';

import {topicStore} from '@/lib/mqttClient';
import {useSyncExternalStore} from 'react';
import {TOPIC} from '@/lib/openmower';

export interface SensorInfo {
  sensor_id: string;
  sensor_name: string;
  value_type: 'STRING' | 'DOUBLE' | 'UNKNOWN';
  value_description: string;
  unit: string;
  has_min_max: boolean | 0 | 1;
  min_value: number;
  max_value: number;
  has_critical_low: boolean | 0 | 1;
  lower_critical_value: number;
  has_critical_high: boolean | 0 | 1;
  upper_critical_value: number;
}

// kept for the whole run, sensor_infos/json is retained and comes only once
const infoStore = topicStore<SensorInfo[]>(TOPIC.sensorInfos, [], (payload) => JSON.parse(payload.toString()));
const valueStore = topicStore<Record<string, string>>(TOPIC.sensorData, {}, (payload, prev, topic) => {
  const id = topic.match(/^sensors\/(.+)\/data$/)?.[1];
  return id ? {...prev, [id]: payload.toString()} : prev;
});

export function useMowerSensors(): {infos: SensorInfo[]; values: Record<string, string>} {
  const infos = useSyncExternalStore(infoStore.subscribe, infoStore.get, infoStore.initial);
  const values = useSyncExternalStore(valueStore.subscribe, valueStore.get, valueStore.initial);
  return {infos, values};
}
