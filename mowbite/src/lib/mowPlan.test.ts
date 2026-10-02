import {describe, expect, it} from 'vitest';
import {polygonArea} from './geometry';
import {fillStripes, linkStripes, mowPlan} from './mowPlan';

const square = (x: number, y: number, size: number) => [
  {x, y},
  {x: x + size, y},
  {x: x + size, y: y + size},
  {x, y: y + size},
];
const base = {holes: [], outlineOffset: 0, outlineCount: 2, overlapCount: 0, toolWidth: 1, angle: 0};

describe('mowPlan', () => {
  it('shrinks the area by the offset first, then by a tool width per pass', () => {
    const plan = mowPlan({...base, outline: square(0, 0, 10), outlineOffset: 0.5});
    expect(plan.loops).toHaveLength(2);
    expect(polygonArea(plan.loops[0])).toBeCloseTo(81); // 9 x 9
    expect(polygonArea(plan.loops[1])).toBeCloseTo(49); // 7 x 7
  });

  it('keeps the stripes inside the innermost pass, or reaches under it with overlap', () => {
    // no offset: the first pass runs along the edge, the second 1 m in, the stripes fill that
    const inside = mowPlan({...base, outline: square(0, 0, 10)});
    for (const [a, b] of inside.stripes) {
      expect(Math.min(a.x, b.x)).toBeGreaterThanOrEqual(1 - 1e-6);
      expect(Math.max(a.x, b.x)).toBeLessThanOrEqual(9 + 1e-6);
    }
    const overlap = mowPlan({...base, outline: square(0, 0, 10), overlapCount: 1});
    expect(Math.min(...overlap.stripes.map(([a, b]) => Math.min(a.x, b.x)))).toBeCloseTo(0);
  });

  it('gives obstacles their own passes and keeps stripes out of them', () => {
    const plan = mowPlan({...base, outline: square(0, 0, 20), holes: [square(8, 8, 4)], outlineCount: 1});
    expect(plan.loops).toHaveLength(2);
    const throughHole = plan.stripes.filter(([a]) => Math.abs(a.y - 10.5) < 1e-6);
    expect(throughHole).toHaveLength(2);
  });

  it('ignores obstacles outside the area and cuts off ones over its edge', () => {
    const plan = mowPlan({...base, outline: square(0, 0, 10), holes: [square(20, 20, 2), square(9, 4, 2)], outlineCount: 1});
    expect(plan.loops).toHaveLength(1);
    expect(polygonArea(plan.loops[0])).toBeCloseTo(98); // 100 minus the 1 x 2 m bit that sticks in
  });

  it('without outline passes the stripes fill the whole area', () => {
    const plan = mowPlan({...base, outline: square(0, 0, 10), outlineCount: 0, outlineOffset: 2});
    expect(plan.loops).toHaveLength(0);
    expect(plan.stripes).toHaveLength(10);
  });

  it('stops when the area is too small for more passes', () => {
    const plan = mowPlan({...base, outline: square(0, 0, 3), outlineCount: 5});
    expect(plan.loops.length).toBeLessThan(5);
  });
});

describe('linkStripes', () => {
  it('joins the stripes of a plain area into one zigzag', () => {
    const stripes = fillStripes([square(0, 0, 5)], 0, 0.5);
    const chains = linkStripes(stripes, 0, 0.5);
    expect(chains).toHaveLength(1);
    expect(chains[0]).toHaveLength(stripes.length * 2);
    // it turns at the ends: every other stripe runs the other way
    expect(chains[0][1].x).toBeCloseTo(5);
    expect(chains[0][2].x).toBeCloseTo(5);
  });

  it('starts a new piece where an obstacle splits the stripes', () => {
    const stripes = fillStripes([square(0, 0, 10), square(4, 2, 2)], 0, 0.5);
    expect(linkStripes(stripes, 0, 0.5).length).toBeGreaterThan(1);
  });
});
