import {afterEach, describe, expect, it, vi} from 'vitest';
import {forget, fresh} from './fresh';

afterEach(() => {
  forget('');
  vi.useRealTimers();
});

describe('fresh', () => {
  it('asks once for requests close together', async () => {
    const load = vi.fn(async () => 42);
    expect(await fresh('a', load)).toBe(42);
    expect(await fresh('a', load)).toBe(42);
    expect(load).toHaveBeenCalledTimes(1);
  });

  it('asks again once the answer is older than maxAge', async () => {
    vi.useFakeTimers();
    const load = vi.fn(async () => 1);
    await fresh('b', load, 1000);
    vi.advanceTimersByTime(1500);
    await fresh('b', load, 1000);
    expect(load).toHaveBeenCalledTimes(2);
  });

  it("doesn't keep failures", async () => {
    const load = vi.fn().mockRejectedValueOnce(new Error('no')).mockResolvedValueOnce(2);
    await expect(fresh('c', load)).rejects.toThrow('no');
    expect(await fresh('c', load)).toBe(2);
  });

  it('asks again after forget', async () => {
    const load = vi.fn(async () => 3);
    await fresh('d:1', load);
    forget('d:');
    await fresh('d:1', load);
    expect(load).toHaveBeenCalledTimes(2);
  });
});
