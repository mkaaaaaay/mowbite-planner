import {describe, expect, it} from 'vitest';
import type {MapArea, MowerMap, Point} from '@/hooks/useMowerMap';
import {checkMap, selfCrossing} from './mapCheck';

const pts = (...xy: number[]): Point[] => xy.flatMap((v, i) => (i % 2 ? [] : [{x: v, y: xy[i + 1]}]));
const area = (id: string, type: string, outline: Point[], active = true): MapArea => ({id, properties: {type, active}, outline});
const square = (x: number, y: number, s: number) => pts(x, y, x + s, y, x + s, y + s, x, y + s);

describe('selfCrossing', () => {
  it('finds nothing in a plain or a concave outline', () => {
    expect(selfCrossing(square(0, 0, 10))).toBeNull();
    expect(selfCrossing(pts(0, 0, 10, 0, 10, 10, 5, 3, 0, 10))).toBeNull();
  });

  it('finds where a figure eight crosses', () => {
    expect(selfCrossing(pts(0, 0, 10, 10, 10, 0, 0, 10))).toEqual({x: 5, y: 5});
  });

  it('finds a point dragged onto another edge', () => {
    expect(selfCrossing(pts(0, 0, 10, 0, 10, 10, 5, 0, 0, 10))).toEqual({x: 5, y: 0});
  });

  it('stays quick for a recorded outline with many points', () => {
    const circle = Array.from({length: 5000}, (_, i) => ({x: Math.cos((i / 5000) * 2 * Math.PI) * 20, y: Math.sin((i / 5000) * 2 * Math.PI) * 20}));
    const t = performance.now();
    expect(selfCrossing(circle)).toBeNull();
    expect(performance.now() - t).toBeLessThan(500);
  });
});

describe('checkMap', () => {
  const lawn = area('lawn', 'mow', square(0, 0, 10));
  // docked facing west, the approach point is 1.5 m east of it
  const map = (areas: MapArea[], dock = {x: 1, y: 1}, heading = Math.PI): MowerMap => ({
    areas,
    docking_stations: [{id: 'd', position: dock, heading}],
  });

  it('is fine with a lawn, a bed in it and the dock on it', () => {
    expect(checkMap(map([lawn, area('bed', 'obstacle', square(2, 2, 2))]))).toEqual([]);
  });

  it('reports a crossing outline and one with too few points', () => {
    const kinds = checkMap(map([lawn, area('eight', 'mow', pts(0, 0, 10, 10, 10, 0, 0, 10)), area('line', 'nav', pts(0, 0, 1, 1))])).map(
      (p) => `${p.kind} ${'areaId' in p ? p.areaId : ''}`,
    );
    expect(kinds).toEqual(['crossing eight', 'points line']);
  });

  it('hints at an obstacle outside every area the mower drives on', () => {
    expect(checkMap(map([lawn, area('far', 'obstacle', square(20, 20, 2))]))).toEqual([{kind: 'outside', level: 'hint', areaId: 'far'}]);
  });

  it('takes a dock just outside the lawn, it docks from the approach point in front of it', () => {
    expect(checkMap(map([lawn], {x: -0.5, y: 1}))).toEqual([]);
  });

  it('warns when the approach point is off the active areas, an inactive one doesn\'t count', () => {
    const facingEast = checkMap(map([lawn], {x: 1, y: 1}, 0));
    expect(facingEast).toEqual([{kind: 'dock', level: 'warn', dockId: 'd', at: {x: -0.5, y: expect.closeTo(1)}, distance: 1.5}]);
    expect(checkMap(map([area('lawn', 'mow', square(0, 0, 10), false)])).map((p) => p.kind)).toEqual(['dock']);
  });

  it('uses the approach distance of the mower', () => {
    expect(checkMap(map([lawn], {x: -0.5, y: 1}), 0.3).map((p) => p.kind)).toEqual(['dock']);
  });

  it('leaves drafts alone', () => {
    expect(checkMap(map([lawn, area('sketch', 'draft', pts(0, 0, 10, 10, 10, 0, 0, 10))]))).toEqual([]);
  });
});
