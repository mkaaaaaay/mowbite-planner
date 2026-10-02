import {locale} from './i18n';

export const dayKey = (d: Date) => `${d.getFullYear()}-${d.getMonth()}-${d.getDate()}`;

// today/yesterday in the app's language, like the weekday names
export function dayLabel(d: Date) {
  const today = new Date();
  const yesterday = new Date(today.getFullYear(), today.getMonth(), today.getDate() - 1);
  const rel = new Intl.RelativeTimeFormat(locale(), {numeric: 'auto'});
  const cap = (t: string) => t.charAt(0).toUpperCase() + t.slice(1);
  if (dayKey(d) === dayKey(today)) return cap(rel.format(0, 'day'));
  if (dayKey(d) === dayKey(yesterday)) return cap(rel.format(-1, 'day'));
  return d.toLocaleDateString(locale(), {weekday: 'short', day: 'numeric', month: 'numeric'});
}

export const clock = (t: number, seconds = false) =>
  new Date(t * 1000).toLocaleTimeString(locale(), {hour: '2-digit', minute: '2-digit', ...(seconds ? {second: '2-digit'} : {})});

export function duration(seconds: number) {
  const m = Math.round(seconds / 60);
  if (m < 1) return '<1 min';
  return m < 60 ? `${m} min` : `${Math.floor(m / 60)} h ${m % 60} min`;
}

// "20260926" (event history file name) -> Date
export const parseDay = (d: string) => new Date(+d.slice(0, 4), +d.slice(4, 6) - 1, +d.slice(6, 8));
