import {forget, fresh} from './fresh';
import {apiBase} from './mowers';

// the mowing schedule, kept by the container (docker/schedule.cgi) as simple lines the scheduler
// script reads: enabled, minbattery, skiprain, tz, one "plan <days> <HH:MM> [areas] [end=HH:MM] [dark=1] [sunset=1]
// [fresh=1]" per start time and "pause <YYYY-MM-DD> all|<areas>" for days off up to and including that day. days
// are 1 = monday .. 7 = sunday like date +%u

export interface Plan {
  days: number[];
  time: string; // HH:MM, local
  // area ids to mow, empty: all active ones (the others get skipped by the scheduler)
  areas: string[];
  // HH:MM, local: a run started by the plan is sent home then
  end?: string;
  // may start and mow in the dark, otherwise it doesn't start then, and goes home at sunset when the end time is later
  dark?: boolean;
  // goes home at sunset at the latest
  sunset?: boolean;
  // drops an interrupted job first, so it begins with the first area instead of carrying on
  fresh?: boolean;
}

// days the schedule leaves out, entirely or some areas, from now up to and including the last day
export interface Pause {
  until: string; // YYYY-MM-DD, local
  areas: 'all' | string[];
}

export interface Schedule {
  enabled: boolean;
  minBattery: number;
  skipRain: boolean;
  skipForecast: boolean;
  // how far ahead the forecast is checked, in hours
  forecastHours: number;
  plans: Plan[];
  pause?: Pause;
}

export const FORECAST_HOURS = [1, 2, 3];
export const EMPTY_SCHEDULE: Schedule = {enabled: false, minBattery: 80, skipRain: true, skipForecast: false, forecastHours: 1, plans: []};

const PATH = '/cgi-bin/schedule';
const endpoint = () => apiBase() + PATH;

export function parseSchedule(text: string): Schedule {
  const s: Schedule = {...EMPTY_SCHEDULE, plans: []};
  for (const line of text.split('\n')) {
    const [key, a, b, ...rest] = line.trim().split(' ');
    if (key === 'enabled') s.enabled = a === '1';
    if (key === 'minbattery') s.minBattery = Number(a) || 0;
    if (key === 'skiprain') s.skipRain = a === '1';
    if (key === 'skipforecast') s.skipForecast = a === '1';
    if (key === 'forecasthours' && FORECAST_HOURS.includes(Number(a))) s.forecastHours = Number(a);
    if (key === 'pause' && a && b) s.pause = {until: a, areas: b === 'all' ? 'all' : b.split(',')};
    if (key === 'plan' && a && b) {
      const plan: Plan = {days: a.split(',').map(Number), time: b, areas: []};
      for (const r of rest) {
        if (r.startsWith('end=')) plan.end = r.slice(4);
        else if (r === 'dark=1') plan.dark = true;
        else if (r === 'sunset=1') plan.sunset = true;
        else if (r === 'fresh=1') plan.fresh = true;
        else if (r) plan.areas = r.split(',');
      }
      s.plans.push(plan);
    }
  }
  return s;
}

// pos: where to ask for the forecast, only sent when that's switched on
export function serializeSchedule(s: Schedule, pos?: {lat: number; lon: number}): string {
  return [
    `enabled ${s.enabled ? 1 : 0}`,
    `minbattery ${Math.round(s.minBattery)}`,
    `skiprain ${s.skipRain ? 1 : 0}`,
    `skipforecast ${s.skipForecast ? 1 : 0}`,
    `forecasthours ${s.forecastHours}`,
    ...(pos ? [`pos ${pos.lat.toFixed(2)} ${pos.lon.toFixed(2)}`] : []),
    `tz ${posixTz()}`,
    ...s.plans
      .filter((p) => p.days.length)
      .map(
        (p) =>
          `plan ${[...p.days].sort().join(',')} ${p.time}${p.areas.length ? ` ${p.areas.join(',')}` : ''}${p.end ? ` end=${p.end}` : ''}${p.dark ? ' dark=1' : ''}${p.sunset ? ' sunset=1' : ''}${p.fresh ? ' fresh=1' : ''}`,
      ),
    ...(s.pause && (s.pause.areas === 'all' || s.pause.areas.length)
      ? [`pause ${s.pause.until} ${s.pause.areas === 'all' ? 'all' : s.pause.areas.join(',')}`]
      : []),
    '',
  ].join('\n');
}

// the last answer, so a page opened again shows it right away while it asks the mower
let lastSchedule: Schedule | null | undefined;
let lastLog: LogEntry[] | undefined;
export const cachedSchedule = () => lastSchedule;
export const cachedScheduleLog = () => lastLog;

// null: not served by the container, no scheduler then
export function loadSchedule(): Promise<Schedule | null> {
  return fresh('schedule', async () => {
    try {
      const res = await fetch(endpoint(), {cache: 'no-store'});
      if (!res.ok || !res.headers.get('content-type')?.includes('text/plain')) return (lastSchedule = null);
      return (lastSchedule = parseSchedule(await res.text()));
    } catch {
      return lastSchedule ?? null;
    }
  });
}

export async function saveSchedule(s: Schedule, pos?: {lat: number; lon: number}): Promise<void> {
  lastSchedule = s;
  forget('schedule');
  const res = await fetch(endpoint(), {method: 'POST', body: serializeSchedule(s, pos)});
  if (!res.ok) throw new Error(`schedule ${res.status}`);
}

export interface LogEntry {
  t: number;
  what: string;
  detail?: string;
}

// what the scheduler writes to its log, in words (translated where it's shown)
export const SCHEDULE_LOG_TEXT: Record<string, string> = {
  started: 'Started',
  start_failed: "The mower didn't react to the start, still idle a minute later",
  waiting_battery: 'Waiting for the battery ({detail} %)',
  skip_battery: 'Not started, battery only at {detail} % after two hours',
  skip_rain: "Not started, the mower's rain sensor was wet",
  skip_forecast: 'Not started, rain was forecast',
  skip_busy: 'Not started, the mower was busy ({detail})',
  skip_emergency: 'Not started, emergency stop was active',
  skip_offline: "Not started, the mower wasn't reachable",
  skip_dark: "Not started, it's dark (hedgehogs)",
  stopped_end: 'Sent home, end time reached',
  stopped_dark: 'Sent home at sunset (hedgehogs)',
  stopped_again: 'Sent home again, it carried on by itself after charging',
  skip_paused: 'Not started, paused for today',
  skip_end: 'Not started, the end time came before the battery was charged',
  paused_area: 'Skipped an area paused for today',
  skipped_area: "Skipped an area that wasn't picked",
  unknown_area: "An area the map doesn't have (an older job?), left it alone",
};

// how a start the schedule tried ended: started, or why not
const START_OUTCOMES = new Set([
  'started',
  'start_failed',
  'skip_battery',
  'skip_rain',
  'skip_forecast',
  'skip_busy',
  'skip_emergency',
  'skip_offline',
  'skip_dark',
  'skip_paused',
  'skip_end',
]);

// The last start the schedule tried, when it didn't come off and it's less than 12 hours ago. A pause for the day
// was asked for, and busy means it was out already, neither is worth a word.
export function missedStart(log: LogEntry[], now = Date.now() / 1000): LogEntry | null {
  let last: LogEntry | null = null;
  for (const l of log) if (START_OUTCOMES.has(l.what) && (!last || l.t > last.t)) last = l;
  if (!last || now - last.t > 12 * 3600) return null;
  return ['started', 'skip_paused', 'skip_busy'].includes(last.what) ? null : last;
}

export function loadScheduleLog(): Promise<LogEntry[]> {
  return fresh('schedule-log', loadLog);
}

async function loadLog(): Promise<LogEntry[]> {
  try {
    const res = await fetch(`${endpoint()}?log`, {cache: 'no-store'});
    if (!res.ok) return lastLog ?? [];
    return (lastLog = (await res.text())
      .split('\n')
      .filter(Boolean)
      .map((l) => {
        const [t, what, detail] = l.split(' ');
        return {t: Number(t), what, detail};
      })
      .reverse());
  } catch {
    return lastLog ?? [];
  }
}

// At night hedgehogs and other animals are out and curl up instead of running away, a start time in
// the dark has to be confirmed on the schedule page. With the sun times of the day it's the real
// night, without them 6 pm to 6 am.
export const NIGHT_FROM = '18:00';
export const NIGHT_TO = '06:00';
export const isNightTime = (time: string, sun?: {rise: string; set: string} | null) =>
  sun ? time >= sun.set || time < sun.rise : time >= NIGHT_FROM || time < NIGHT_TO;

// the next start as a date, looking a week ahead from now. days paused for all areas don't count, and with the
// sun times neither do starts in the dark that the scheduler leaves out (not marked dark)
export function nextStart(s: Schedule, from = new Date(), sun?: {rise: string; set: string} | null): Date | null {
  if (!s.enabled) return null;
  let best: Date | null = null;
  for (let add = 0; add <= 7; add++) {
    const day = new Date(from.getFullYear(), from.getMonth(), from.getDate() + add);
    const dow = ((day.getDay() + 6) % 7) + 1;
    // a pause of the whole schedule leaves those days out
    if (s.pause?.areas === 'all' && localDay(day) <= s.pause.until) continue;
    for (const p of s.plans) {
      if (!p.days.includes(dow)) continue;
      if (sun !== undefined && !p.dark && isNightTime(p.time, sun)) continue;
      const [h, m] = p.time.split(':').map(Number);
      const at = new Date(day.getFullYear(), day.getMonth(), day.getDate(), h, m);
      if (at > from && (!best || at < best)) best = at;
    }
  }
  return best;
}

// The browser's time zone as a POSIX TZ string, which is what the container's date understands
// (it has no zoneinfo). The daylight saving rule is read off this year's two switches.
export function posixTz(): string {
  const year = new Date().getFullYear();
  const off = (t: number) => -new Date(t).getTimezoneOffset(); // minutes east of UTC
  const pad = (n: number) => String(n).padStart(2, '0');
  const name = (m: number) => `<${m < 0 ? '-' : '+'}${pad(Math.floor(Math.abs(m) / 60))}${pad(Math.abs(m) % 60)}>`;
  // posix counts west as positive
  const offset = (m: number) => {
    const a = Math.abs(m);
    return `${m > 0 ? '-' : ''}${Math.floor(a / 60)}${a % 60 ? `:${pad(a % 60)}` : ''}`;
  };
  const start = Date.UTC(year, 0, 1);
  const end = Date.UTC(year + 1, 0, 1);
  const switches: {at: number; from: number; to: number}[] = [];
  let prev = off(start);
  for (let t = start; t < end; t += 3600e3) {
    const o = off(t);
    if (o !== prev) switches.push({at: t, from: prev, to: o});
    prev = o;
  }
  if (switches.length !== 2) return `${name(off(start))}${offset(off(start))}`;
  const std = Math.min(off(start), off(Date.UTC(year, 6, 1)));
  const dst = Math.max(off(start), off(Date.UTC(year, 6, 1)));
  const rule = (sw: {at: number; from: number}) => {
    // wall clock time just before the switch
    const local = new Date(sw.at + sw.from * 60e3);
    const month = local.getUTCMonth() + 1;
    const day = local.getUTCDate();
    const daysInMonth = new Date(Date.UTC(local.getUTCFullYear(), month, 0)).getUTCDate();
    const week = day + 7 > daysInMonth ? 5 : Math.ceil(day / 7);
    const min = local.getUTCMinutes();
    return `M${month}.${week}.${local.getUTCDay()}/${local.getUTCHours()}${min ? `:${pad(min)}` : ''}`;
  };
  const toDst = switches.find((s) => s.to === dst)!;
  const toStd = switches.find((s) => s.to === std)!;
  return `${name(std)}${offset(std)}${name(dst)}${offset(dst)},${rule(toDst)},${rule(toStd)}`;
}

// today as YYYY-MM-DD in local time, the scheduler compares it with its own date in the browser's time zone
export function localDay(d = new Date()): string {
  return `${d.getFullYear()}-${String(d.getMonth() + 1).padStart(2, '0')}-${String(d.getDate()).padStart(2, '0')}`;
}
