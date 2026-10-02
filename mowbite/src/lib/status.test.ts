import {describe, expect, it} from 'vitest';
import {gpsQuality} from './status';

describe('gpsQuality', () => {
  it('tells fix, float and too inaccurate by the accuracy, against the mower limit', () => {
    expect(gpsQuality(0.019)).toBe('fix');
    expect(gpsQuality(0.05)).toBe('fix');
    expect(gpsQuality(0.12)).toBe('float');
    expect(gpsQuality(0.2)).toBe('float');
    expect(gpsQuality(0.35)).toBe('poor');
    expect(gpsQuality(0.35, 0.5)).toBe('float');
  });

  it('knows no fix from the 999 the positioning sends, or nothing at all', () => {
    expect(gpsQuality(999)).toBe('none');
    expect(gpsQuality(undefined)).toBe('none');
  });
});
