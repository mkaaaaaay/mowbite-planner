'use client';

import {TitleMark} from '@/components/Logo';
import {clock, dayKey, dayLabel, duration, parseDay} from '@/lib/dates';
import {describe, eventSource, explain, groupRuns, rawLine, OUTCOMES, withState, type Entry, type MowerEvent, type Run} from '@/lib/events';
import {useDragScroll} from '@/hooks/useDragScroll';
import {eventsOfDay, historyDays, localDays, type DayEvents} from '@/lib/history';
import {RosLogAround} from '@/components/RosLog';
import Link from 'next/link';
import {useEffect, useState} from 'react';
import styles from './page.module.css';
import {fmt, tr, useLang} from '@/lib/i18n';
import {PARAM, PATHS} from '@/lib/openmower';
import {numParam, useMowerParams} from '@/hooks/useMowerParams';


function Timeline({events}: {events: MowerEvent[]}) {
  return (
    <ol className={styles.timeline}>
      {withState(events).map(({event, state}) => {
        const {text, severity} = describe(event, state);
        return (
          <li key={event.id} className={styles[severity]}>
            <span className={styles.time}>{clock(event.t, true)}</span>
            <span>{text}</span>
          </li>
        );
      })}
    </ol>
  );
}

// the events carry a position, so we can at least tell whether it moved and whether gps was there
function problemFacts(run: Run, e: MowerEvent, next?: Run): string[] {
  const facts: string[] = [];
  const first = run.events[0];
  if (e.x !== undefined && first.x !== undefined && e.y !== undefined && first.y !== undefined) {
    const d = Math.hypot(e.x - first.x, e.y - first.y);
    facts.push(d < 0.1 ? tr("Hadn't moved ({cm} cm)", {cm: Math.round(d * 100)}) : tr('{m} m from where the run started', {m: fmt(d, 1)}));
  }
  const gps = run.events.filter((g) => g.type === 'GPS' && g.t <= e.t).pop();
  facts.push(gps ? (gps.available ? tr('GPS fix') : tr('no GPS fix')) : tr('GPS still off'));
  if (e.type === 'UNDOCKING_FAILED' && next && next.start - e.t < 600) {
    const worked = next.outcome !== 'undock_failed';
    const gap = next.start - e.t;
    const after = gap < 60 ? `${Math.round(gap)} s` : duration(gap);
    facts.push(worked ? tr('Tried again {time} later, that worked', {time: after}) : tr('Tried again {time} later, failed again', {time: after}));
  }
  return facts;
}

// the mower keeps one file per day, relative to its ros folder
type Where = DayEvents['where'];

function RawEntry({event, file, line}: {event: MowerEvent; file: string; line: number}) {
  const src = eventSource(event);
  return (
    <div className={styles.raw}>
      <div className={styles.dim}>
        {file}
        {line > 0 && tr(', line {n}', {n: line})}
      </div>
      <code>{rawLine(event)}</code>
      {src && (
        <div className={styles.dim}>
          {tr('Written by')}{' '}
          <a href={src.url} target="_blank" rel="noreferrer">
            open_mower_ros/{src.file.split('/').pop()}
          </a>
          {src.log && (
            <>
              , {tr('ROS log')}: <code>{src.log}</code>
            </>
          )}
        </div>
      )}
    </div>
  );
}

function ProblemItem({run, next, event, state, where}: {run: Run; next?: Run; event: MowerEvent; state?: string; where: Where}) {
  const [raw, setRaw] = useState(false);
  const {text, severity} = describe(event, state);
  const params = useMowerParams();
  const hint = explain(event, state, numParam(params, PARAM.mowerLogic('gps_timeout')));
  return (
    <li className={styles[severity]}>
      <div>
        <span className={styles.time}>{clock(event.t, true)}</span>
        <strong>{text}</strong>
      </div>
      {hint && <p>{hint}</p>}
      <p className={styles.facts}>{problemFacts(run, event, next).join(' · ')}</p>
      <div className={styles.problemActions}>
        {event.x !== undefined && event.y !== undefined && (
          <Link
            className={styles.onMap}
            href={
              `/map?at=${event.x.toFixed(2)},${event.y.toFixed(2)}` +
              (run.jobId ? `&job=${run.jobId}` : '') +
              `&msg=${encodeURIComponent(`${clock(event.t, true)} ${text}`)}`
            }
          >
            <svg viewBox="0 0 24 24" aria-hidden="true">
              <path d="M12 21s-7-6.2-7-11.5a7 7 0 0 1 14 0C19 14.8 12 21 12 21Z" />
              <circle cx="12" cy="9.5" r="2.5" />
            </svg>
            {tr('Show on map')}
          </Link>
        )}
        <button className={styles.rawToggle} onClick={() => setRaw(!raw)}>
          {raw ? tr('Hide entry') : tr('Show entry')}
        </button>
        <RosLogAround t={event.t} />
      </div>
      {raw && <RawEntry event={event} file={`${PATHS.eventHistory}${where.get(event.id)?.file ?? ''}`} line={where.get(event.id)?.line ?? 0} />}
    </li>
  );
}

function Problems({run, next, where}: {run: Run; next?: Run; where: Where}) {
  const list = withState(run.events).filter(({event, state}) => describe(event, state).severity !== 'info');
  if (!list.length) return null;
  return (
    <ul className={styles.problemList}>
      {list.map(({event, state}) => (
        <ProblemItem
          key={event.id}
          run={run}
          next={next}
          event={event}
          state={state}
          where={where}
        />
      ))}
    </ul>
  );
}

function RunCard({run, next, where}: {run: Run; next?: Run; where: Where}) {
  const [open, setOpen] = useState(false);
  const outcome = OUTCOMES[run.outcome];
  return (
    <article className={[styles.run, styles[`tone-${outcome.tone}`]].join(' ')}>
      <header onClick={() => setOpen(!open)}>
        <div className={styles.runTime}>
          <strong>
            {clock(run.start)} – {clock(run.end)}
          </strong>
          <span className={styles.dim}>{duration(run.end - run.start)}</span>
        </div>
        <span className={styles.badge}>{tr(outcome.label)}</span>
      </header>

      <div className={styles.runFacts}>
        {run.areas.map((a) => (
          <span key={a} className={styles.area}>
            {a}
          </span>
        ))}
        {run.bladeSeconds > 0 && <span className={styles.dim}>{tr('mowed {time}', {time: duration(run.bladeSeconds)})}</span>}
      </div>

      <Problems run={run} next={next} where={where} />

      <div className={styles.runActions}>
        <button onClick={() => setOpen(!open)}>{open ? tr('Hide details') : tr('Details')}</button>
        {run.jobId && <Link href={`/map?job=${run.jobId}`}>{tr('Show track')}</Link>}
      </div>

      {open && <Timeline events={run.events} />}
    </article>
  );
}

// what was loaded before, shown right away when the page is opened again
let cachedFiles: string[] = [];
let cachedDays: string[] | null = null;
const cachedEvents = new Map<string, DayEvents>();

export default function ActivityPage() {
  useLang();
  const [days, setDays] = useState<string[] | null>(cachedDays);
  const [day, setDay] = useState<string | null>(null);
  const [loaded, setLoaded] = useState<DayEvents | null>(() => cachedEvents.get(cachedDays?.[0] ?? '') ?? null);
  const events = loaded?.events ?? null;
  const setEvents = (d: DayEvents | null) => setLoaded(d);
  const [onlyProblems, setOnlyProblems] = useState(false);
  const [failed, setFailed] = useState(false);
  const daysRef = useDragScroll<HTMLDivElement>();

  useEffect(() => {
    historyDays().then(
      (files) => {
        cachedFiles = files;
        cachedDays = localDays(files);
        setDays(cachedDays);
        setDay((d) => d ?? cachedDays?.[0] ?? null);
      },
      () => setFailed(true),
    );
  }, []);

  const shown = day ?? days?.[0] ?? null;
  const isToday = !!shown && dayKey(parseDay(shown)) === dayKey(new Date());

  useEffect(() => {
    if (!shown) return;
    let alive = true;
    const load = () =>
      eventsOfDay(cachedFiles, shown).then(
        (e) => {
          cachedEvents.set(shown, e);
          if (alive) setEvents(e);
        },
        () => alive && setFailed(true),
      );
    void load();
    // today keeps growing, the mower has no live event topic, so ask again now and then
    const timer = isToday ? setInterval(load, 30000) : undefined;
    return () => {
      alive = false;
      if (timer) clearInterval(timer);
    };
  }, [shown, isToday]);

  // outside of runs only start/shutdown and real problems are interesting, not gps toggling in the dock
  const entries: Entry[] = (events ? groupRuns(events, isToday).reverse() : []).filter(
    (e) => e.kind === 'run' || ['BOOTED', 'SHUTDOWN'].includes(e.event.type) || describe(e.event).severity !== 'info',
  );
  // which file and line an entry came from, the day's early or late hours sit in the neighbouring file
  const where = loaded?.where ?? new Map<string, {file: string; line: number}>();
  const runs = entries.flatMap((e) => (e.kind === 'run' ? [e.run] : []));
  const mowed = runs.reduce((s, r) => s + r.bladeSeconds, 0);
  const problems = runs.reduce((s, r) => s + r.problems, 0);
  const visible = entries.filter((e) =>
    !onlyProblems ? true : e.kind === 'run' ? e.run.problems > 0 : describe(e.event).severity !== 'info',
  );
  // messages between runs that follow each other share one card
  const blocks: ({kind: 'run'; run: Run} | {kind: 'loose'; events: MowerEvent[]})[] = [];
  for (const e of visible) {
    const last = blocks[blocks.length - 1];
    if (e.kind === 'run') blocks.push(e);
    else if (last?.kind === 'loose') last.events.push(e.event);
    else blocks.push({kind: 'loose', events: [e.event]});
  }

  return (
    <div className={styles.page}>
      <main className={styles.main}>
        <h1>
          <TitleMark />
          {tr('Activity')}
        </h1>

        {failed && !days && <p className={styles.dim}>{tr("This mower doesn't keep an event history.")}</p>}

        {days && (
          <div className={styles.days} ref={daysRef}>
            {days.slice(0, 60).map((d) => (
              <button
                key={d}
                className={d === shown ? styles.on : undefined}
                onClick={() => {
                  setEvents(null);
                  setDay(d);
                }}
              >
                {dayLabel(parseDay(d))}
              </button>
            ))}
          </div>
        )}

        {events && (
          <div className={styles.summary}>
            <div>
              <strong>{duration(mowed)}</strong>
              <span>{tr('mowed')}</span>
            </div>
            <div>
              <strong>{runs.length}</strong>
              <span>{runs.length === 1 ? tr('run') : tr('runs')}</span>
            </div>
            <div className={problems ? styles.bad : undefined}>
              <strong>{problems}</strong>
              <span>{problems === 1 ? tr('problem') : tr('problems')}</span>
            </div>
            <label className={styles.filter}>
              <input type="checkbox" checked={onlyProblems} onChange={() => setOnlyProblems(!onlyProblems)} />
              {tr('only problems')}
            </label>
          </div>
        )}

        {shown && !events && <p className={styles.dim}>{tr('loading…')}</p>}
        {events && visible.length === 0 && <p className={styles.dim}>{tr('Nothing here.')}</p>}

        <div className={styles.list}>
          {blocks.map((b) =>
            b.kind === 'run' ? (
              <RunCard key={b.run.events[0].id} run={b.run} next={runs[runs.indexOf(b.run) - 1]} where={where} />
            ) : (
              <div key={b.events[0].id} className={styles.looseGroup}>
                {b.events.map((ev) => (
                  <div key={ev.id} className={[styles.loose, styles[describe(ev).severity]].join(' ')}>
                    <span className={styles.time}>{clock(ev.t)}</span>
                    {describe(ev).text}
                  </div>
                ))}
              </div>
            ),
          )}
        </div>
      </main>
    </div>
  );
}
