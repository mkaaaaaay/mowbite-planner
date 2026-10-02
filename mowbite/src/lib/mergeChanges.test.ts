import {describe, expect, it} from 'vitest';
import {mergeChanges} from './mergeChanges';

describe('mergeChanges', () => {
  const base = {colors: {mower: '#f00', dock: '#00f'}, grass: true, mowers: [{id: 'a'}]};

  it('keeps what the other device changed and puts mine on top', () => {
    const theirs = {...base, colors: {...base.colors, dock: '#0f0'}, weather: true};
    const mine = {...base, colors: {...base.colors, mower: '#fff'}};
    expect(mergeChanges(theirs, base, mine)).toEqual({colors: {mower: '#fff', dock: '#0f0'}, grass: true, mowers: [{id: 'a'}], weather: true});
  });

  it('takes mine where both changed the same thing, lists as a whole', () => {
    const theirs = {...base, grass: false, mowers: [{id: 'a'}, {id: 'b'}]};
    const mine = {...base, grass: true, mowers: [{id: 'c'}]};
    expect(mergeChanges(theirs, {...base, grass: false}, mine).grass).toBe(true);
    expect(mergeChanges(theirs, base, mine).mowers).toEqual([{id: 'c'}]);
  });

  it('removes what I removed', () => {
    const mine: Partial<typeof base> = {...base};
    delete mine.grass;
    expect('grass' in mergeChanges(base, base, mine)).toBe(false);
  });
});
