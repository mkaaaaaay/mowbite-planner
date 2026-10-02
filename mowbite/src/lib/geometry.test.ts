import {describe, expect, it} from 'vitest';
import {containsPoint, polygonArea} from './geometry';
import {mergeOutlines} from './mergeAreas';
import {simplifyPolygon} from './simplifyPolygon';
import {splitByPath} from './splitPolygon';

const square = (x: number, y: number, size: number) => [
  {x, y},
  {x: x + size, y},
  {x: x + size, y: y + size},
  {x, y: y + size},
];

describe('polygonArea / containsPoint', () => {
  it('measures and hit-tests a square either way round', () => {
    expect(polygonArea(square(0, 0, 10))).toBe(100);
    expect(polygonArea([...square(0, 0, 10)].reverse())).toBe(100);
    expect(containsPoint(square(0, 0, 10), 5, 5)).toBe(true);
    expect(containsPoint(square(0, 0, 10), 11, 5)).toBe(false);
  });
});

describe('splitByPath', () => {
  it('cuts a square in two halves along a straight line', () => {
    const parts = splitByPath(square(0, 0, 10), [
      {x: 5, y: -1},
      {x: 5, y: 11},
    ]);
    expect(parts).not.toBeNull();
    expect(parts!.map(polygonArea).sort()).toEqual([50, 50]);
  });

  it('follows a bend in the cut', () => {
    const parts = splitByPath(square(0, 0, 10), [
      {x: 2, y: -1},
      {x: 2, y: 5},
      {x: 8, y: 5},
      {x: 8, y: 11},
    ]);
    expect(parts).not.toBeNull();
    const [a, b] = parts!.map(polygonArea);
    expect(a + b).toBeCloseTo(100);
    expect(a).toBeCloseTo(50);
  });

  it('refuses a line that misses the area', () => {
    expect(splitByPath(square(0, 0, 10), [{x: 20, y: -1}, {x: 20, y: 11}])).toBeNull();
  });
});

describe('mergeOutlines', () => {
  it('joins two overlapping squares', () => {
    const merged = mergeOutlines(square(0, 0, 10), square(5, 0, 10));
    expect(merged).not.toBeNull();
    expect(polygonArea(merged!.outline)).toBeCloseTo(150);
    expect(merged!.holesFilled).toBe(0);
  });

  it("won't join areas that don't touch", () => {
    expect(mergeOutlines(square(0, 0, 10), square(20, 0, 10))).toBeNull();
  });
});

describe('simplifyPolygon', () => {
  it('drops points that sit on a straight edge', () => {
    // a square with a point every 10 cm
    const dense = [];
    for (let i = 0; i < 100; i++) dense.push({x: i / 10, y: 0});
    for (let i = 0; i < 100; i++) dense.push({x: 10, y: i / 10});
    for (let i = 0; i < 100; i++) dense.push({x: 10 - i / 10, y: 10});
    for (let i = 0; i < 100; i++) dense.push({x: 0, y: 10 - i / 10});
    const simple = simplifyPolygon(dense, 0.05);
    expect(simple.length).toBeLessThanOrEqual(5);
    expect(polygonArea(simple)).toBeCloseTo(100);
  });

  it('keeps small shapes as they are', () => {
    const s = square(0, 0, 1);
    expect(simplifyPolygon(s, 1)).toBe(s);
  });
});
