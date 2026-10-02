import {describe, expect, it} from 'vitest';
import {polygonArea} from './geometry';
import {cutOut, splitByPath} from './splitPolygon';

const square = (x: number, y: number, s: number) => [
  {x, y},
  {x: x + s, y},
  {x: x + s, y: y + s},
  {x, y: y + s},
];

describe('cutOut', () => {
  it('cuts a shape inside out and splits the rest in two', () => {
    const outline = square(0, 0, 10);
    const pieces = cutOut(outline, square(4, 4, 2));
    expect(pieces).not.toBeNull();
    const [a, b, inner] = pieces!;
    expect(polygonArea(inner)).toBeCloseTo(4);
    expect(polygonArea(a) + polygonArea(b)).toBeCloseTo(96);
  });

  it('cuts across the narrow side', () => {
    const outline = [
      {x: 0, y: 0},
      {x: 20, y: 0},
      {x: 20, y: 4},
      {x: 0, y: 4},
    ];
    const [a, b] = cutOut(outline, square(9, 1, 2))!;
    // a cut along x would leave halves of 20 m, across it they are about 9 m wide
    const width = (o: {x: number}[]) => Math.max(...o.map((p) => p.x)) - Math.min(...o.map((p) => p.x));
    expect(width(a)).toBeLessThan(12);
    expect(width(b)).toBeLessThan(12);
  });

  it('needs the shape fully inside', () => {
    expect(cutOut(square(0, 0, 10), square(8, 8, 4))).toBeNull();
  });

  it('refuses a shape that crosses itself', () => {
    const bow = [
      {x: 2, y: 2},
      {x: 6, y: 6},
      {x: 6, y: 2},
      {x: 2, y: 6},
    ];
    expect(cutOut(square(0, 0, 10), bow)).toBeNull();
  });

  it('leaves a line through the area to the normal split', () => {
    expect(splitByPath(square(0, 0, 10), [{x: -1, y: 5}, {x: 11, y: 5}])).not.toBeNull();
    expect(cutOut(square(0, 0, 10), [{x: -1, y: 5}, {x: 11, y: 5}, {x: 5, y: 8}])).toBeNull();
  });
});
