import {describe, expect, it} from 'vitest';
import type {MapArea, MowerMap} from '@/hooks/useMowerMap';
import {mergeMaps} from './mergeMap';

const area = (id: string, name: string, x = 0): MapArea => ({id, properties: {name, type: 'mow'}, outline: [{x, y: 0}, {x: x + 1, y: 0}, {x, y: 1}]});
const map = (...areas: MapArea[]): MowerMap => ({areas, docking_stations: [{id: 'd', position: {x: 0, y: 0}, heading: 0}]});
const names = (m: MowerMap) => m.areas.map((a) => a.properties.name);

describe('mergeMaps', () => {
  const base = map(area('a', 'Front'), area('b', 'Back'), area('c', 'Side'));

  it('takes my change of one area and their change of another', () => {
    const mine = map(area('a', 'Front lawn'), area('b', 'Back'), area('c', 'Side'));
    const theirs = map(area('a', 'Front'), area('b', 'Back', 5), area('c', 'Side'));
    const {map: m, conflicts} = mergeMaps(base, mine, theirs);
    expect(names(m)).toEqual(['Front lawn', 'Back', 'Side']);
    expect(m.areas[1].outline[0].x).toBe(5);
    expect(conflicts).toEqual([]);
  });

  it('keeps an area recorded on the mower and one drawn here, each after its neighbour', () => {
    const mine = map(area('a', 'Front'), area('n', 'Drawn'), area('b', 'Back'), area('c', 'Side'));
    const theirs = map(area('a', 'Front'), area('b', 'Back'), area('c', 'Side'), area('r', 'Recorded'));
    expect(names(mergeMaps(base, mine, theirs).map)).toEqual(['Front', 'Drawn', 'Back', 'Side', 'Recorded']);
  });

  it('removes what one side deleted and the other left alone', () => {
    const mine = map(area('a', 'Front'), area('c', 'Side'));
    const theirs = map(area('a', 'Front'), area('b', 'Back'));
    expect(names(mergeMaps(base, mine, theirs).map)).toEqual(['Front']);
  });

  it('reports an area both changed differently and keeps mine there', () => {
    const mine = map(area('a', 'Mine'), area('b', 'Back'), area('c', 'Side'));
    const theirs = map(area('a', 'Theirs'), area('b', 'Back'), area('c', 'Side'));
    const {map: m, conflicts} = mergeMaps(base, mine, theirs);
    expect(names(m)[0]).toBe('Mine');
    expect(conflicts).toEqual(['a']);
  });

  it('keeps a new mowing order from either side', () => {
    const reordered = map(area('c', 'Side'), area('a', 'Front'), area('b', 'Back'));
    expect(names(mergeMaps(base, reordered, base).map)).toEqual(['Side', 'Front', 'Back']);
    expect(names(mergeMaps(base, base, reordered).map)).toEqual(['Side', 'Front', 'Back']);
  });

  it('takes a docking station moved on the mower', () => {
    const theirs = {...base, docking_stations: [{id: 'd', position: {x: 2, y: 1}, heading: 1}]};
    expect(mergeMaps(base, map(area('a', 'X'), area('b', 'Back'), area('c', 'Side')), theirs).map.docking_stations[0].position).toEqual({x: 2, y: 1});
  });
});
