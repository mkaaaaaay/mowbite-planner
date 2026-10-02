import {describe, expect, it} from 'vitest';
import {stripeAngleDiff} from './mowDirection';
import {angleInRange, autoMowAngle} from './mowStripes';

describe('autoMowAngle', () => {
  it('points from the first outline point to the first one more than 2 m away, like the mower', () => {
    const outline = [
      {x: 0, y: 0},
      {x: 1, y: 1},
      {x: 0, y: 3},
      {x: 5, y: 5},
    ];
    expect(autoMowAngle(outline)).toBeCloseTo(Math.PI / 2);
    expect(autoMowAngle([{x: 0, y: 0}, {x: 1, y: 0}])).toBe(0);
  });
});

describe('stripeAngleDiff', () => {
  it('treats stripes as having no front', () => {
    expect(stripeAngleDiff(10, 170)).toBe(-20);
    expect(stripeAngleDiff(170, 10)).toBe(20);
    expect(stripeAngleDiff(0, 90)).toBe(-90);
  });
});

describe('angleInRange', () => {
  it('leaves the angle alone without a range or inside it', () => {
    expect(angleInRange(0.9)).toBe(0.9);
    expect(angleInRange(0.3, 0.2, 0.5)).toBeCloseTo(0.3);
  });

  it('bounces back at the ends, and holds it with min = max', () => {
    expect(angleInRange(0.6, 0.2, 0.5)).toBeCloseTo(0.4);
    expect(angleInRange(0.9, 0.2, 0.5)).toBeCloseTo(0.3);
    expect(angleInRange(-0.2269, 0.2, 0.5)).toBeCloseTo(0.3731);
    expect(angleInRange(2, 1, 1)).toBe(1);
  });

  const deg = Math.PI / 180;
  const inDeg = (a: number) => Math.round(a / deg);

  it("doesn't care how the angle is written", () => {
    expect(inDeg(angleInRange(-350 * deg, 0, 50 * deg))).toBe(10);
    expect(inDeg(angleInRange(-5 * deg, 0, 50 * deg))).toBe(5);
  });

  it('keeps an angle whose stripes are in the range, 0° and 180° are the same stripes', () => {
    expect(inDeg(angleInRange(200 * deg, 0, 50 * deg))).toBe(20);
    expect(inDeg(angleInRange(-160 * deg, 0, 50 * deg))).toBe(20);
    expect(inDeg(angleInRange(100 * deg, 0, 50 * deg))).toBe(0);
  });

  it('leaves the angle alone with half a turn or more', () => {
    expect(angleInRange(2, 0, Math.PI)).toBe(2);
    expect(angleInRange(2, -170 * deg, 170 * deg)).toBe(2);
  });

  it('takes max < min as a range across ±180°', () => {
    expect(inDeg(angleInRange(-175 * deg, 170 * deg, -170 * deg))).toBe(185);
    expect(inDeg(angleInRange(160 * deg, 170 * deg, -170 * deg))).toBe(180);
  });

  it('gives the same as the mower for random angles', () => {
    // the mower's version (MowingBehavior.cpp), std::remainder written out
    const mower = (angle: number, lo: number, hi: number) => {
      const d = hi - lo;
      if (d >= Math.PI) return angle;
      let width = d % Math.PI;
      if (width < 0) width += Math.PI;
      if (width === 0) return lo;
      const mid = lo + width / 2;
      const x = angle - mid;
      angle = mid + (x - Math.PI * Math.round(x / Math.PI));
      let t = (angle - lo) % (2 * width);
      if (t < 0) t += 2 * width;
      return lo + (t <= width ? t : 2 * width - t);
    };
    let seed = 1;
    const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647) * 8 * Math.PI - 4 * Math.PI;
    for (let i = 0; i < 20000; i++) {
      const lo = rnd() / 4;
      const hi = rnd() / 4;
      const a = rnd();
      const d = angleInRange(a, lo, hi) - mower(a, lo, hi);
      expect(Math.abs(Math.sin(d / 2))).toBeLessThan(1e-9);
    }
  });
});
