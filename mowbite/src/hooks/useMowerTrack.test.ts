import {describe, expect, it} from 'vitest';
import {appendPoint, chunked, thin, trackPoints, type Point, type TrackChunks} from './useMowerTrack';

// a mowing pattern with turns, blade changes and jitter, so plenty of points survive the thinning
function drive(n: number): Point[] {
  const out: Point[] = [];
  let seed = 7;
  const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647 - 0.5) * 0.04;
  for (let i = 0; i < n; i++) {
    const lane = Math.floor(i / 60);
    const along = i % 60;
    out.push({x: (lane % 2 ? 59 - along : along) * 0.05 + rnd(), y: lane * 0.18 + rnd(), b: i % 500 < 450});
  }
  return out;
}

const flat = (chunks: TrackChunks) => chunks.flatMap((c, i) => (i ? c.slice(1) : c));

describe('live trail in pieces', () => {
  it('draws the same as the old single list, piece by piece', () => {
    const raw = drive(12000);
    let chunks: TrackChunks = [];
    for (const p of raw) chunks = appendPoint(chunks, p) ?? chunks;
    const old = thin(raw);
    expect(old.length).toBeGreaterThan(2000);
    expect(flat(chunks)).toEqual(old);
    expect(trackPoints(chunks)).toBe(old.length);
  });

  it('pieces stay small and join up', () => {
    let chunks: TrackChunks = [];
    for (const p of drive(8000)) chunks = appendPoint(chunks, p) ?? chunks;
    expect(chunks.length).toBeGreaterThan(3);
    for (let i = 0; i < chunks.length; i++) {
      expect(chunks[i].length).toBeLessThanOrEqual(400);
      if (i) expect(chunks[i][0]).toBe(chunks[i - 1][chunks[i - 1].length - 1]);
    }
  });

  it('finished pieces keep their identity when a point is added', () => {
    let chunks: TrackChunks = [];
    for (const p of drive(8000)) chunks = appendPoint(chunks, p) ?? chunks;
    const next = appendPoint(chunks, {x: 50, y: 50, b: true})!;
    for (let i = 0; i < chunks.length - 1; i++) expect(next[i]).toBe(chunks[i]);
  });

  it('drops the oldest pieces past the limit', () => {
    let chunks: TrackChunks = [];
    for (const p of drive(12000)) chunks = appendPoint(chunks, p, 1500) ?? chunks;
    expect(trackPoints(chunks)).toBeLessThanOrEqual(1500);
    expect(flat(chunks)).toEqual(thin(drive(12000)).slice(-trackPoints(chunks)));
  });

  it('cuts a loaded trail the same way', () => {
    const old = thin(drive(9000));
    const chunks = chunked(old);
    expect(flat(chunks)).toEqual(old);
    expect(chunked([])).toEqual([]);
    expect(flat(chunked(old.slice(0, 1)))).toEqual(old.slice(0, 1));
  });
});
