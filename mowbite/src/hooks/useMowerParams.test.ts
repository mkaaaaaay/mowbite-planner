import {describe, expect, it} from 'vitest';
import {numParam} from './useMowerParams';

describe('numParam', () => {
  const sent = {'/mower_logic/outline_overlap_count': 1, '/mower_logic/tool_width': 0.18};

  it('takes what the mower sent', () => {
    expect(numParam(sent, '/mower_logic/tool_width')).toBe(0.18);
    expect(numParam(sent, '/mower_logic/outline_overlap_count')).toBe(1);
  });

  it("falls back to mower_logic's default for one the list came without", () => {
    expect(numParam(sent, '/mower_logic/outline_count')).toBe(3);
    expect(numParam({'/mower_logic/outline_overlap_count': 1}, '/mower_logic/tool_width')).toBe(0.14);
  });

  it('knows nothing for params without a default', () => {
    expect(numParam(sent, '/ll/services/gps/datum_lat')).toBeUndefined();
  });
});
