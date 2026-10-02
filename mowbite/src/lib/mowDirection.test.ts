import {describe, expect, it} from 'vitest';
import type {TrackSegment} from '@/hooks/useMowHistory';
import {measuredStripeAngle, stripeAngleDiff} from './mowDirection';

const rect = [
  {x: 1, y: -3},
  {x: 7, y: -3},
  {x: 7, y: 6},
  {x: 1, y: 6},
];

// a closed lap inset by d, sampled every 10 cm
function lap(d: number): [number, number][] {
  const c: [number, number][] = [
    [1 + d, -3 + d],
    [7 - d, -3 + d],
    [7 - d, 6 - d],
    [1 + d, 6 - d],
    [1 + d, -3 + d],
  ];
  const pts: [number, number][] = [];
  for (let i = 0; i < 4; i++) {
    const [ax, ay] = c[i];
    const [bx, by] = c[i + 1];
    const n = Math.round(Math.hypot(bx - ax, by - ay) / 0.1);
    for (let k = 0; k < n; k++) pts.push([ax + ((bx - ax) * k) / n, ay + ((by - ay) * k) / n]);
  }
  return pts;
}

// horizontal stripes through the inside
function stripes(): [number, number][] {
  const pts: [number, number][] = [];
  for (let y = -1; y <= 4; y += 0.5) {
    const dir = Math.round(y * 2) % 2 === 0 ? 1 : -1;
    for (let k = 0; k <= 40; k++) pts.push([dir > 0 ? 2 + k * 0.1 : 6 - k * 0.1, y]);
  }
  return pts;
}

const seg = (points: [number, number][]): TrackSegment => ({points, attributes: {blades: true}}) as unknown as TrackSegment;

describe('measuredStripeAngle', () => {
  it('ignores outline passes even when the pass count is unknown', () => {
    expect(measuredStripeAngle([seg(lap(0.3)), seg(lap(0.5)), seg(lap(0.7))], rect, 0.2)).toBeNull();
  });
  it('finds the stripe direction', () => {
    const r = measuredStripeAngle([seg(lap(0.3)), seg(stripes())], rect, 0.2);
    expect(r).not.toBeNull();
    expect(Math.abs(stripeAngleDiff(r!.angleDeg, 0))).toBeLessThan(2);
  });
});
