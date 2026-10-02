import {describe, expect, it} from 'vitest';
import type {MowerEvent} from './events';
import {eventsOfDay, fileDay, localDays} from './history';

process.env.TZ = 'Europe/Berlin';

const ev = (id: string, local: string): MowerEvent => ({id, type: 'STATE', t: new Date(local).getTime() / 1000});

describe('event history days', () => {
  it('adds today when only yesterday has a file yet', () => {
    const today = fileDay(new Date());
    const y = new Date();
    y.setDate(y.getDate() - 1);
    expect(localDays([fileDay(y), '20260901'])).toEqual([today, fileDay(y), '20260901']);
    expect(localDays(['20260901'])).toEqual(['20260901']);
  });
  it('takes the early hours of a day from the previous file (east of utc)', async () => {
    const files: Record<string, MowerEvent[]> = {
      // utc day 28 holds 02:00 local of the 28th up to 01:59 local of the 29th
      '20260928': [ev('a', '2026-09-28T10:00:00+02:00'), ev('b', '2026-09-29T00:53:00+02:00')],
      '20260929': [ev('c', '2026-09-29T09:00:00+02:00')],
    };
    const load = (f: string) => Promise.resolve(files[f] ?? []);
    const day29 = await eventsOfDay(Object.keys(files), '20260929', load);
    expect(day29.events.map((e) => e.id)).toEqual(['b', 'c']);
    expect(day29.where.get('b')).toEqual({file: '20260928.jsonl', line: 2});
    const day28 = await eventsOfDay(Object.keys(files), '20260928', load);
    expect(day28.events.map((e) => e.id)).toEqual(['a']);
  });
});
