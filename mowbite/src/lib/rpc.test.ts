import {EventEmitter} from 'node:events';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import {isMissingMethod, RpcError, unavailable} from './rpc';

describe('isMissingMethod', () => {
  it('spots the json-rpc "method not found" error only', () => {
    expect(isMissingMethod(new RpcError('map.replace', -32601, 'Method not found'))).toBe(true);
    expect(isMissingMethod(new RpcError('map.replace', 'timeout', 'timed out'))).toBe(false);
    expect(isMissingMethod(new Error('Method not found'))).toBe(false);
  });
});

describe('unavailable', () => {
  const known = new Set(['map.replace', 'events.history']);

  it('stops calls the mower said it does not have', () => {
    expect(unavailable(known, 'position.history')).toBe(true);
    expect(unavailable(known, 'map.replace')).toBe(false);
  });

  it('never blocks when the mower could not tell, nor rpc.methods and meta.* calls', () => {
    expect(unavailable(null, 'position.history')).toBe(false);
    expect(unavailable(known, 'rpc.methods')).toBe(false);
    expect(unavailable(known, 'meta.config.schema')).toBe(false);
  });
});

describe('calls while offline', () => {
  // stands in for the mqtt client: what gets published is kept, a subscribe is confirmed right away
  class FakeClient extends EventEmitter {
    connected = false;
    published: {method: string; id: string}[] = [];
    publish(_topic: string, payload: string) {
      this.published.push(JSON.parse(payload));
    }
    subscribe(_topic: string, cb?: () => void) {
      cb?.();
    }
    online() {
      this.connected = true;
      this.emit('connect');
    }
  }

  let client: FakeClient;
  // a fresh rpc module per test, it asks rpc.methods only once per page load
  const load = async () => {
    vi.resetModules();
    vi.doMock('./mqttClient', () => ({
      getMqttClient: () => client,
      withPrefix: (t: string) => t,
      onTopic: (topic: string, h: (p: Buffer, t: string) => void) => {
        client.on('message', (t: string, p: Buffer) => t === topic && h(p, t));
        return () => {};
      },
    }));
    return import('./rpc');
  };

  beforeEach(() => {
    client = new FakeClient();
    vi.useFakeTimers();
  });
  afterEach(() => {
    vi.doUnmock('./mqttClient');
    vi.useRealTimers();
  });

  it('waits for the connection, then sends and takes the answer', async () => {
    const {rpcMethods} = await load();
    const methods = rpcMethods();
    expect(client.published).toEqual([]);
    client.online();
    expect(client.published.map((m) => m.method)).toEqual(['rpc.methods']);
    client.emit('message', 'rpc/response', Buffer.from(JSON.stringify({id: client.published[0].id, result: ['map.replace']})));
    expect(await methods).toEqual(new Set(['map.replace']));
  });

  it('asks for the method list again a while after it got no answer', async () => {
    const {rpcMethods} = await load();
    const first = rpcMethods();
    await vi.advanceTimersByTimeAsync(6000);
    expect(await first).toBeNull();
    client.online();
    // right after, the failed answer still counts
    expect(await rpcMethods()).toBeNull();
    expect(client.published).toEqual([]);
    await vi.advanceTimersByTimeAsync(15000);
    const again = rpcMethods();
    expect(client.published.map((m) => m.method)).toEqual(['rpc.methods']);
    client.emit('message', 'rpc/response', Buffer.from(JSON.stringify({id: client.published[0].id, result: ['mowing.plan']})));
    expect(await again).toEqual(new Set(['mowing.plan']));
  });

  it('keeps a "method not found" from an older mower, that one is final', async () => {
    const {rpcMethods} = await load();
    client.online();
    const first = rpcMethods();
    client.emit('message', 'rpc/response', Buffer.from(JSON.stringify({id: client.published[0].id, error: {code: -32601, message: 'Method not found'}})));
    expect(await first).toBeNull();
    await vi.advanceTimersByTimeAsync(60000);
    await rpcMethods();
    expect(client.published).toHaveLength(1);
  });

  it('never sends a call that timed out before the connection came back', async () => {
    const {rpcMethods} = await load();
    const methods = rpcMethods();
    await vi.advanceTimersByTimeAsync(6000);
    expect(await methods).toBeNull();
    client.online();
    expect(client.published).toEqual([]);
  });
});
