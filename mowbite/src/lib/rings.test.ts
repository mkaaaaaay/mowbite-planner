import {describe, expect, it} from 'vitest';
import type {MowerMap} from '@/hooks/useMowerMap';
import {closedRings, closeRing, openRing, openRings} from './rings';

const tri = [
  {x: 0, y: 0},
  {x: 1, y: 0},
  {x: 0, y: 1},
];

describe('rings', () => {
  it('opens a closed outline and closes an open one, once', () => {
    expect(openRing([...tri, {x: 0, y: 0}])).toEqual(tri);
    expect(openRing(tri)).toEqual(tri);
    expect(closeRing(tri)).toEqual([...tri, {x: 0, y: 0}]);
    expect(closeRing(closeRing(tri))).toEqual([...tri, {x: 0, y: 0}]);
  });

  it('closes every area of a map for saving and gets the same map back when opened', () => {
    const map = {areas: [{id: 'a', properties: {}, outline: tri}], docking_stations: []} as MowerMap;
    const saved = closedRings(map);
    const o = saved.areas[0].outline;
    expect(o[0]).toEqual(o[o.length - 1]);
    expect(openRings(saved)).toEqual(map);
  });
});
