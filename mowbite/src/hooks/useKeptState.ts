'use client';

import {useEffect, useState} from 'react';

// like useState, but the value outlives the page while the app runs (until a reload), e.g. edits that
// shouldn't be gone after a look at another tab
const kept = new Map<string, unknown>();

export function useKeptState<T>(key: string, initial: T) {
  const state = useState<T>(() => (kept.has(key) ? (kept.get(key) as T) : initial));
  const value = state[0];
  useEffect(() => {
    kept.set(key, value);
  }, [key, value]);
  return state;
}
