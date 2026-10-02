'use client';

import {TitleMark} from '@/components/Logo';
import {datumFromParams, useMowerParams} from '@/hooks/useMowerParams';
import {sunTimes} from '@/lib/sun';
import {clock, dayLabel} from '@/lib/dates';
import {tr, useLang} from '@/lib/i18n';
import {useMowerActions} from '@/hooks/useMowerActions';
import {ACTION} from '@/lib/openmower';
import PlanRow from './PlanRow';
import PauseCard from './PauseCard';
import {
  cachedSchedule,
  cachedScheduleLog,
  EMPTY_SCHEDULE,
  FORECAST_HOURS,
  loadSchedule,
  loadScheduleLog,
  nextStart,
  saveSchedule,
  type LogEntry,
  type Schedule,
  SCHEDULE_LOG_TEXT,
} from '@/lib/schedule';
import {useEffect, useState} from 'react';
import styles from './page.module.css';
import {useMowerMap} from '@/hooks/useMowerMap';

export default function SchedulePage() {
  useLang();
  const params = useMowerParams();
  const map = useMowerMap();
  // the areas a plan can pick from: mowing areas the mower actually mows
  const mowAreas = (map?.areas ?? []).filter(
    (a) => a.properties.type === 'mow' && a.properties.active !== false && a.properties.mowable !== false,
  );
  const datum = datumFromParams(params);
  // today's sunrise and sunset, so the animal hint follows the real night
  const sun = datum ? sunTimes(datum.lat, datum.lon) : null;
  // undefined: loading, null: not served by the container
  const [schedule, setSchedule] = useState<Schedule | null | undefined>(cachedSchedule);
  const [log, setLog] = useState<LogEntry[]>(() => cachedScheduleLog() ?? []);
  const [error, setError] = useState<string | null>(null);
  const {knowsAction} = useMowerActions();

  useEffect(() => {
    void loadSchedule().then(setSchedule);
    void loadScheduleLog().then(setLog);
    const timer = setInterval(() => void loadScheduleLog().then(setLog), 60000);
    return () => clearInterval(timer);
  }, []);

  // every change is saved right away
  const update = (next: Schedule) => {
    setSchedule(next);
    setError(null);
    // the position also gives the scheduler the sun times
    saveSchedule(next, datum ?? undefined).catch((e) =>
      setError(e instanceof Error ? e.message : tr('failed')),
    );
  };

  const s = schedule ?? EMPTY_SCHEDULE;
  const next = nextStart(s, undefined, sun);

  return (
    <div className={styles.page}>
      <main className={styles.main}>
        <h1>
          <TitleMark />
          {tr('Schedule')}
        </h1>

        {schedule === null && (
          <p className={styles.dim}>{tr('The schedule needs MowBite running as its container on the mower.')}</p>
        )}

        {schedule && (
          <>
            <section className={styles.card}>
              <label className={styles.switch}>
                <input type="checkbox" checked={s.enabled} onChange={(e) => update({...s, enabled: e.target.checked})} />
                <strong>{tr('Mow by schedule')}</strong>
              </label>
              <p className={styles.dim}>
                {next
                  ? tr('Next start: {when}', {when: `${dayLabel(next)}, ${clock(next.getTime() / 1000)}`})
                  : s.enabled
                    ? tr('No start planned yet.')
                    : tr('Switched off, the mower only mows when you start it.')}
              </p>
            </section>

            {s.plans.length > 0 && <PauseCard schedule={s} mowAreas={mowAreas} update={update} />}

            <section className={styles.card}>
              <h2>{tr('Start times')}</h2>
              <p className={styles.dim}>
                {tr('Each start time has its own days, times and areas. For a plan per area, add a start time for each and pick only that area.')}
              </p>
              {s.plans.length === 0 && <p className={styles.dim}>{tr('No start times yet.')}</p>}
              {s.plans.map((p, i) => (
                <PlanRow
                  key={i}
                  plan={p}
                  sun={sun}
                  mowAreas={mowAreas}
                  canReset={knowsAction(ACTION.resetJob)}
                  onChange={(q) => update({...s, plans: s.plans.map((x, j) => (j === i ? q : x))})}
                  onRemove={() => update({...s, plans: s.plans.filter((_, j) => j !== i)})}
                />
              ))}
              <button className={styles.add} onClick={() => update({...s, plans: [...s.plans, {days: [1, 3, 5], time: '10:00', areas: []}]})}>
                + {tr('Add start time')}
              </button>
            </section>

            <section className={styles.card}>
              <h2>{tr('Conditions')}</h2>
              <label className={styles.slider}>
                <span>{s.minBattery > 0 ? tr('Battery at least {n} %', {n: s.minBattery}) : tr("Don't wait for the battery")}</span>
                <input
                  type="range"
                  min={0}
                  max={100}
                  step={5}
                  value={s.minBattery}
                  onChange={(e) => update({...s, minBattery: Number(e.target.value)})}
                />
              </label>
              <p className={styles.dim}>
                {tr("If it isn't charged that far at the start time, it waits for up to two hours and then leaves it for that day.")}
              </p>
              <label className={styles.check}>
                <input type="checkbox" checked={s.skipRain} onChange={(e) => update({...s, skipRain: e.target.checked})} />
                {tr("Not when the mower's rain sensor is wet")}
              </label>
              <label className={styles.check}>
                <input
                  type="checkbox"
                  checked={s.skipForecast}
                  disabled={!datum}
                  onChange={(e) => update({...s, skipForecast: e.target.checked})}
                />
                {tr('Not when it rains or rain is forecast for the next')}
                <select
                  className={styles.hours}
                  value={s.forecastHours}
                  disabled={!datum}
                  onChange={(e) => update({...s, forecastHours: Number(e.target.value)})}
                >
                  {FORECAST_HOURS.map((h) => (
                    <option key={h} value={h}>
                      {h === 1 ? tr('hour') : tr('{n} hours', {n: h})}
                    </option>
                  ))}
                </select>
              </label>
              <p className={styles.dim}>
                {tr('The forecast comes from Open-Meteo, the mower asks for it with the position rounded to about a kilometer.')}
              </p>
              <p className={styles.dim}>{tr('The mower is only started when it stands idle and no emergency stop is active.')}</p>
              {mowAreas.length > 1 && (
                <p className={styles.dim}>
                  {tr(
                    'With only some areas picked, the mower still starts as usual and skips the others when it gets to them, in the normal mowing order.',
                  )}
                </p>
              )}
            </section>

            <section className={styles.card}>
              <h2>{tr('What happened')}</h2>
              {log.length === 0 && <p className={styles.dim}>{tr('Nothing yet.')}</p>}
              <ul className={styles.log}>
                {log.slice(0, 20).map((l, i) => (
                  <li key={i} className={l.what === 'started' ? styles.good : l.what.startsWith('skip') || l.what === 'start_failed' ? styles.bad : undefined}>
                    <span className={styles.dim}>
                      {dayLabel(new Date(l.t * 1000))}, {clock(l.t)}
                    </span>
                    {tr(SCHEDULE_LOG_TEXT[l.what] ?? l.what, {detail: l.detail ?? ''})}
                  </li>
                ))}
              </ul>
            </section>

            {error && <p className={styles.error}>{error}</p>}
          </>
        )}
      </main>
    </div>
  );
}
