import {describe, expect, it} from 'vitest';
import {overrideError} from './editing';

describe('overrideError', () => {
  it('takes empty (global value) and values within the mower limits', () => {
    expect(overrideError('outline_count', '')).toBeNull();
    expect(overrideError('outline_count', '3')).toBeNull();
    expect(overrideError('outline_overlap_count', '0')).toBeNull();
    expect(overrideError('outline_offset', '-0.25')).toBeNull();
    expect(overrideError('outline_offset', '1')).toBeNull();
  });

  it('turns down what the mower would ignore or misread', () => {
    // -2 would count as "not set", 1.5 as 1
    expect(overrideError('outline_count', '-2')).not.toBeNull();
    expect(overrideError('outline_count', '1.5')).not.toBeNull();
    expect(overrideError('outline_overlap_count', '300')).not.toBeNull();
    expect(overrideError('outline_offset', '5')).not.toBeNull();
    expect(overrideError('outline_offset', '-1.2')).not.toBeNull();
  });
});
