'use client';

import {getMqttClient, topicStore, withPrefix} from '@/lib/mqttClient';
import {useCallback, useSyncExternalStore} from 'react';
import {TOPIC} from '@/lib/openmower';

interface ActionInfo {
  action_id: string;
  action_name: string;
  // 1/0, not a bool
  enabled: boolean | 0 | 1;
}

// kept for the whole run, actions/json is retained and comes only once
const store = topicStore<Record<string, boolean>>(TOPIC.actions, {}, (payload) => {
  const list: ActionInfo[] = JSON.parse(payload.toString());
  return Object.fromEntries(list.map((a) => [a.action_id, !!a.enabled]));
});

export function useMowerActions(): {
  hasAction: (id: string) => boolean;
  // the mower has it, enabled right now or not
  knowsAction: (id: string) => boolean;
  publishAction: (id: string) => void;
} {
  const actions = useSyncExternalStore(store.subscribe, store.get, store.initial);

  const hasAction = useCallback((id: string) => !!actions[id], [actions]);
  const knowsAction = useCallback((id: string) => id in actions, [actions]);

  // plain text, not json
  const publishAction = useCallback((id: string) => {
    getMqttClient().publish(withPrefix(TOPIC.action), id);
  }, []);

  return {hasAction, knowsAction, publishAction};
}
