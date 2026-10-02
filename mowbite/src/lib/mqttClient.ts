import mqtt, {type MqttClient} from 'mqtt';
import {selectedMower} from './mowers';

declare global {
  interface Window {
    // from /config.js, written by the container on start
    __MOWER_CONFIG__?: {mqttUrl?: string; mqttPrefix?: string};
  }
}

function config() {
  return typeof window === 'undefined' ? {} : (window.__MOWER_CONFIG__ ?? {});
}

// another mower picked in the app, or the one it runs on. that falls back to the host the app is
// served from, which is the mower itself when installed there
function mqttUrl(): string {
  const other = selectedMower();
  if (other) return `ws://${other.host}:${other.wsPort ?? 9001}`;
  return config().mqttUrl || process.env.NEXT_PUBLIC_MOWER_MQTT_WS_URL || `ws://${window.location.hostname}:9001`;
}

function topicPrefix(): string {
  const other = typeof window === 'undefined' ? null : selectedMower();
  return other ? (other.prefix ?? '') : (config().mqttPrefix ?? '');
}

export function withPrefix(topic: string): string {
  return topicPrefix() + topic;
}

// topic without the prefix, null if it's not one of ours
export function unprefix(topic: string): string | null {
  const prefix = topicPrefix();
  return topic.startsWith(prefix) ? topic.slice(prefix.length) : null;
}

// one shared connection for the whole app
let client: MqttClient | null = null;

export function getMqttClient(): MqttClient {
  if (!client) {
    client = mqtt.connect(mqttUrl(), {
      reconnectPeriod: 2000,
      // a dead wifi link shows up after ~15 s instead of ~90 s with the default of 60
      keepalive: 10,
      // nothing sent while offline is kept for later: a start or go home tapped then would go out on the
      // next reconnect, maybe minutes later. rpc waits for the connection itself (lib/rpc.ts)
      queueQoSZero: false,
      // the topics are subscribed again below, once each
      resubscribe: false,
    });
    client.on('connect', () => {
      if (handlers.size) client!.subscribe([...handlers.keys()].map(withPrefix));
    });
    client.on('message', (full, payload) => {
      const topic = unprefix(full);
      if (topic === null) return;
      for (const [t, set] of handlers) if (t === topic || matches(t, topic)) set.forEach((h) => h(payload, topic));
    });
  }
  return client;
}

// One listener for the whole app hands each message to whoever asked for its topic (without the prefix, + for one
// level). Every topic is subscribed once, and again after a reconnect. The subscriptions stay when the last one
// stops listening, retained topics like the map wouldn't come again otherwise.
export type TopicHandler = (payload: Buffer, topic: string) => void;
const handlers = new Map<string, Set<TopicHandler>>();

const matches = (pattern: string, topic: string) => {
  if (!pattern.includes('+')) return false;
  const p = pattern.split('/');
  const t = topic.split('/');
  return p.length === t.length && p.every((part, i) => part === '+' || part === t[i]);
};

export function onTopic(topic: string, handler: TopicHandler): () => void {
  const c = getMqttClient();
  let set = handlers.get(topic);
  if (!set) {
    handlers.set(topic, (set = new Set()));
    // offline the connect above does it
    if (c.connected) c.subscribe(withPrefix(topic));
  }
  set.add(handler);
  return () => set.delete(handler);
}

// A topic's latest value kept for the whole time the app runs, for useSyncExternalStore. Retained topics (map,
// actions) only come once per subscription, so a page that opens later still needs what came before. parse gets
// the previous value too, for topics with + that build one value from many messages
export function topicStore<T>(topic: string, initial: T, parse: (payload: Buffer, prev: T, topic: string) => T) {
  let value = initial;
  let started = false;
  const listeners = new Set<() => void>();
  return {
    subscribe(listener: () => void) {
      if (!started) {
        started = true;
        onTopic(topic, (payload, t) => {
          try {
            value = parse(payload, value, t);
          } catch {
            return; // malformed payload
          }
          listeners.forEach((l) => l());
        });
      }
      listeners.add(listener);
      return () => listeners.delete(listener);
    },
    get: () => value,
    initial: () => initial,
  };
}

