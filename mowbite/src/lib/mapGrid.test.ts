import {describe, expect, it} from 'vitest';
import {handleRadius, meterGrid} from './mapGrid';

describe('meterGrid', () => {
  it('picks the finest step that keeps lines 25px apart', () => {
    expect(meterGrid({left: 0, right: 10, bottom: 0, top: 10}, 60).step).toBe(0.5);
    expect(meterGrid({left: 0, right: 10, bottom: 0, top: 10}, 20).step).toBe(2);
    expect(meterGrid({left: 0, right: 10, bottom: 0, top: 10}, 0.1).step).toBe(50);
  });

  it('puts lines on whole steps inside the view', () => {
    const g = meterGrid({left: -3.2, right: 3.2, bottom: 0.5, top: 4.9}, 20);
    expect(g.xs).toEqual([-2, 0, 2]);
    expect(g.ys).toEqual([2, 4]);
  });
});

describe('handleRadius', () => {
  const ring = (n: number, r: number) =>
    Array.from({length: n}, (_, i) => ({x: r * Math.cos((2 * Math.PI * i) / n), y: r * Math.sin((2 * Math.PI * i) / n)}));

  it('is full size for points far apart and shrinks for dense outlines', () => {
    expect(handleRadius(ring(4, 10), 20)).toBe(2.5);
    expect(handleRadius(ring(2000, 10), 20)).toBe(1.5);
  });
});
