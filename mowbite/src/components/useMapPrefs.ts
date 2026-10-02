'use client';

import type {ImagerySource} from '@/lib/imagery';
import {useState} from 'react';
import type {Layer, PlanStyle} from './MapControls';

function read<T>(key: string, parse: (v: string | null) => T, fallback: T): T {
  try {
    return parse(localStorage.getItem(key));
  } catch {
    return fallback;
  }
}

function write(key: string, value: string | null) {
  try {
    if (value === null) localStorage.removeItem(key);
    else localStorage.setItem(key, value);
  } catch {}
}

// How the map is shown on this device: aerial imagery, the grid, the layers switched off in the menu and the style
// of what's left of a plan. Only used once the map arrived, client side, so localStorage is there.
export function useMapPrefs() {
  // off by default, it sends the map area to the imagery provider
  const [imagery, setImageryState] = useState(() => read('mapImagery', (v) => (v as ImagerySource | null) || null, null));
  const [showGrid, setShowGrid] = useState(() => read('mapGrid', (v) => v !== 'off', true));
  const [hidden, setHidden] = useState(() => read('mapHidden', (v) => new Set<Layer>(JSON.parse(v ?? '[]')), new Set<Layer>()));
  const [planStyle, setPlanStyleState] = useState(() =>
    read<PlanStyle>('planStyle', (v) => (v === 'solid' || v === 'dots' ? v : 'dashed'), 'dashed'),
  );
  return {
    imagery,
    setImagery: (next: ImagerySource | null) => {
      setImageryState(next);
      write('mapImagery', next);
    },
    showGrid,
    toggleGrid: () => {
      setShowGrid(!showGrid);
      write('mapGrid', showGrid ? 'off' : 'on');
    },
    hidden,
    toggleLayer: (l: Layer) => {
      const next = new Set(hidden);
      if (next.has(l)) next.delete(l);
      else next.add(l);
      setHidden(next);
      write('mapHidden', JSON.stringify([...next]));
    },
    planStyle,
    setPlanStyle: (st: PlanStyle) => {
      setPlanStyleState(st);
      write('planStyle', st);
    },
  };
}
