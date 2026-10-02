import {describe, expect, it} from 'vitest';
import {seasonOf, sunTimes} from './sun';

process.env.TZ = 'Europe/Berlin';

const minutes = (t: string) => Number(t.slice(0, 2)) * 60 + Number(t.slice(3));
const near = (got: string, want: string) => Math.abs(minutes(got) - minutes(want)) <= 3;

describe('sunTimes', () => {
  // Hanover, checked against a second implementation of the sunrise equation
  it('late september', () => {
    const s = sunTimes(52.37, 9.73, new Date(2026, 8, 29))!;
    expect(near(s.rise, '07:19')).toBe(true);
    expect(near(s.set, '19:05')).toBe(true);
  });
  it('midsummer', () => {
    const s = sunTimes(52.37, 9.73, new Date(2026, 5, 21))!;
    expect(near(s.rise, '04:59')).toBe(true);
    expect(near(s.set, '21:47')).toBe(true);
  });
  it('midwinter', () => {
    const s = sunTimes(52.37, 9.73, new Date(2026, 11, 21))!;
    expect(near(s.rise, '08:29')).toBe(true);
    expect(near(s.set, '16:09')).toBe(true);
  });
  it('polar night gives null', () => {
    expect(sunTimes(78.2, 15.6, new Date(2026, 11, 21))).toBeNull();
  });
});

describe('seasonOf', () => {
  it('in the north', () => {
    expect(seasonOf(52.4, new Date(2026, 9, 1))).toBe('autumn');
    expect(seasonOf(52.4, new Date(2026, 11, 1))).toBe('winter');
    expect(seasonOf(52.4, new Date(2027, 1, 28))).toBe('winter');
    expect(seasonOf(52.4, new Date(2026, 2, 1))).toBe('spring');
    expect(seasonOf(52.4, new Date(2026, 6, 15))).toBe('summer');
  });
  it('the other way round in the south', () => {
    expect(seasonOf(-33.9, new Date(2026, 3, 15))).toBe('autumn');
    expect(seasonOf(-33.9, new Date(2026, 6, 15))).toBe('winter');
    expect(seasonOf(-33.9, new Date(2026, 9, 1))).toBe('spring');
    expect(seasonOf(-33.9, new Date(2026, 0, 1))).toBe('summer');
  });
  it('the north without a position', () => {
    expect(seasonOf(undefined, new Date(2026, 10, 30))).toBe('autumn');
    expect(seasonOf(undefined, new Date(2026, 11, 1))).toBe('winter');
  });
});
