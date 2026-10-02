import {describe, expect, it} from 'vitest';
import {chargeSamples, drainSamples, parseBattery, trend} from './battery';

describe('battery', () => {
  const cycles = parseBattery('charge 100 6000 98.3 24.10 28.00\nrun 6000 7100 30.0 28.00 26.50\nrun 7100 7200 2 26.5 26.4\ncharge 7200 7500 5 26.4 28\nbad line\n');
  it('reads what battery.sh writes', () => {
    expect(cycles).toHaveLength(4);
    expect(cycles[0]).toEqual({kind: 'charge', start: 100, end: 6000, minutes: 98.3, from: 24.1, to: 28});
  });
  it('leaves out top ups and short drives', () => {
    expect(chargeSamples(cycles)).toEqual([{t: 6000e3, v: 98.3}]);
    expect(drainSamples(cycles)).toEqual([{t: 7100e3, v: 3}]);
  });
  it('shows a trend only after some weeks', () => {
    const day = 86400e3;
    const s = Array.from({length: 60}, (_, i) => ({t: i * day, v: 100 - i / 2}));
    expect(trend(s.slice(0, 20), 20 * day)).toBeNull();
    const t = trend(s, 60 * day)!;
    expect(t.first).toBeGreaterThan(t.last);
  });
});
