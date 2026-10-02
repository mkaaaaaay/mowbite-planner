import {useSyncExternalStore} from 'react';
import de from './de';

// The English text is the key, a language file maps it to its translation. Anything missing just
// stays English. The language is per device, by default the browser's.

export type Lang = 'en' | 'de';
export type LangChoice = 'auto' | Lang;

const DICTS: Record<Lang, Record<string, string>> = {en: {}, de};
const KEY = 'lang';
const listeners = new Set<() => void>();

function readChoice(): LangChoice {
  try {
    const v = localStorage.getItem(KEY);
    if (v === 'en' || v === 'de') return v;
  } catch {}
  return 'auto';
}

function detect(): Lang {
  return typeof navigator !== 'undefined' && navigator.language?.toLowerCase().startsWith('de') ? 'de' : 'en';
}

let choice: LangChoice | null = null;
// what tr() uses. It follows the snapshot react renders with, so the first render after the static
// html (always english) matches it and the switch happens right after
let active: Lang = 'en';

export function langChoice(): LangChoice {
  choice ??= readChoice();
  return choice;
}

export function setLangChoice(c: LangChoice) {
  choice = c;
  try {
    if (c === 'auto') localStorage.removeItem(KEY);
    else localStorage.setItem(KEY, c);
  } catch {}
  snapshot();
  listeners.forEach((l) => l());
}

function snapshot(): Lang {
  const c = langChoice();
  active = c === 'auto' ? detect() : c;
  if (document.documentElement.lang !== active) document.documentElement.lang = active;
  return active;
}

function serverSnapshot(): Lang {
  active = 'en';
  return 'en';
}

function subscribe(l: () => void) {
  listeners.add(l);
  return () => listeners.delete(l);
}

// components call this so they render again when the language changes
export function useLang(): Lang {
  return useSyncExternalStore(subscribe, snapshot, serverSnapshot);
}

// what's picked in the settings, auto included
export function useLangChoice(): LangChoice {
  return useSyncExternalStore(subscribe, langChoice, () => 'auto');
}

export function tr(text: string, vars?: Record<string, string | number>): string {
  const s = DICTS[active][text] ?? text;
  return vars ? s.replace(/\{(\w+)\}/g, (m, k) => (k in vars ? String(vars[k]) : m)) : s;
}

// for dates and numbers
// en-GB for english: day before month and a 24 h clock, and not the browser's own language
export const locale = () => (active === 'de' ? 'de-DE' : 'en-GB');

// a number with a fixed count of decimals, decimal comma in german
export const fmt = (v: number, digits = 0) =>
  v.toLocaleString(locale(), {minimumFractionDigits: digits, maximumFractionDigits: digits, useGrouping: false});
