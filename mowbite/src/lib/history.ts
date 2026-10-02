import {dayKey, parseDay} from './dates';
import type {MowerEvent} from './events';
import {fresh} from './fresh';
import {callRpc} from './rpc';
import {RPC} from './openmower';

// the mower's event history, shared by the dashboard and the activity page
export const historyDays = () => fresh('events:list', () => callRpc<string[]>(RPC.eventDays));

export const historyOf = (date: string) =>
  fresh(`events:${date}`, () => callRpc<MowerEvent[]>(RPC.events, {date}, 20000));

// The mower names its files by the day in UTC, the app shows local days. So the first or last hours
// of a local day sit in the file next to it, depending on which side of UTC the time zone is.

export const fileDay = (d: Date) =>
  `${d.getFullYear()}${String(d.getMonth() + 1).padStart(2, '0')}${String(d.getDate()).padStart(2, '0')}`;

const shift = (day: string, by: number) => {
  const d = parseDay(day);
  d.setDate(d.getDate() + by);
  return fileDay(d);
};

// +1 east of UTC (the early hours of a local day are in the previous file), -1 west of it
const side = () => Math.sign(-new Date().getTimezoneOffset());

// the local days worth showing, newest first: the file days, plus today when the newest file is
// today's or yesterday's and today's own file isn't there yet
export function localDays(files: string[]): string[] {
  const days = new Set(files);
  const today = fileDay(new Date());
  if (files.length && !days.has(today) && (files.includes(shift(today, -1)) || files.includes(shift(today, 1)))) days.add(today);
  return [...days].sort().reverse();
}

export interface DayEvents {
  events: MowerEvent[]; // in time order
  where: Map<string, {file: string; line: number}>; // event id -> its file and line
}

// the events of a local day, from its file and the neighbour that holds the rest
export async function eventsOfDay(files: string[], day: string, load = historyOf): Promise<DayEvents> {
  const wanted = dayKey(parseDay(day));
  const from = [day, side() ? shift(day, -side()) : ''].filter((f) => files.includes(f));
  const events: MowerEvent[] = [];
  const where = new Map<string, {file: string; line: number}>();
  for (const f of from) {
    const list = await load(f);
    list.forEach((e, i) => {
      where.set(e.id, {file: `${f}.jsonl`, line: i + 1});
      if (dayKey(new Date(e.t * 1000)) === wanted) events.push(e);
    });
  }
  events.sort((a, b) => a.t - b.t);
  return {events, where};
}
