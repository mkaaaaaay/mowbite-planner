import {describe, expect, it} from 'vitest';
import type {MapArea} from '@/hooks/useMowerMap';
import {mowAroundHoles} from './mowAround';

const square = (x: number, y: number, s: number) => [
  {x, y},
  {x: x + s, y},
  {x: x + s, y: y + s},
  {x, y: y + s},
];
const area = (id: string, outline: ReturnType<typeof square>, properties: MapArea['properties'] = {}): MapArea =>
  ({id, outline, properties: {type: 'mow', ...properties}}) as MapArea;

describe('mowAroundHoles', () => {
  const lawn = area('lawn', square(0, 0, 10));

  it('leaves out a don\'t mow area with mow_around', () => {
    expect(mowAroundHoles(lawn, [lawn, area('bed', square(2, 2, 2), {mowable: false, mow_around: true})])).toHaveLength(1);
  });

  it('keeps the lanes across one without mow_around', () => {
    expect(mowAroundHoles(lawn, [lawn, area('bed', square(2, 2, 2), {mowable: false})])).toHaveLength(0);
  });

  it('ignores one covering the whole area, inactive ones and mowable ones', () => {
    const meadow = area('meadow', square(-5, -5, 30), {mowable: false, mow_around: true});
    const off = area('off', square(2, 2, 2), {mowable: false, mow_around: true, active: false});
    const mowed = area('mowed', square(5, 5, 2), {mow_around: true});
    expect(mowAroundHoles(lawn, [lawn, meadow, off, mowed])).toHaveLength(0);
  });
});
