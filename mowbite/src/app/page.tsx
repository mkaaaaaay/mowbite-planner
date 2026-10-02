'use client';

import LogoMark from '@/components/Logo';
import LoveMower from '@/components/LoveMower';
import MapView from '@/components/MapView';
import {HomeIcon, PlayIcon, SkipIcon, StopIcon, WarningIcon} from '@/components/icons';
import {useComputedSpeed} from '@/hooks/useComputedSpeed';
import {useMowerActions} from '@/hooks/useMowerActions';
import {useMowerMap} from '@/hooks/useMowerMap';
import {datumFromParams, numParam, useMowerParams} from '@/hooks/useMowerParams';
import {sunTimes} from '@/lib/sun';
import {useMowerSensors} from '@/hooks/useMowerSensors';
import {useMowerPosition} from '@/hooks/useMowerPosition';
import {isLive, useMowerState, type MowerState} from '@/hooks/useMowerState';
import {useMowerTrack} from '@/hooks/useMowerTrack';
import {usePlanProgress} from '@/hooks/usePlanProgress';
import {useRecentRuns} from '@/hooks/useRecentRuns';
import {useWeather} from '@/hooks/useWeather';
import WeatherIcon, {WEATHER_LABELS, weatherKind} from '@/components/WeatherIcon';
import {clock, dayKey, dayLabel, duration} from '@/lib/dates';
import {OUTCOMES, type MowerEvent, type Run} from '@/lib/events';
import {settingsStore} from '@/lib/settings';
import {batteryColor, GPS_QUALITY_LABEL, gpsQuality, isDocked} from '@/lib/status';
import Link from 'next/link';
import {useEffect, useState, useSyncExternalStore} from 'react';
import styles from './page.module.css';
import {fmt, tr, useLang} from '@/lib/i18n';
import {
  cachedSchedule,
  cachedScheduleLog,
  loadSchedule,
  loadScheduleLog,
  missedStart,
  nextStart,
  SCHEDULE_LOG_TEXT,
  type LogEntry,
  type Schedule,
} from '@/lib/schedule';
import MowerSwitch from '@/components/MowerSwitch';
import {UpdateBanner} from '@/components/Updates';
import {ACTION, PARAM} from '@/lib/openmower';

// paused counts too, the mower is standing somewhere on the lawn then
const DRIVING = new Set(['MOWING', 'PAUSED', 'DOCKING', 'UNDOCKING']);

const ACTION_RESET_EMERGENCY = ACTION.resetEmergency;
// skipping an area drops what's left of it, so it goes out only after a few seconds and a second tap takes it back
const SKIP_DELAY = 4;
// the left out start that was closed, by its time
const MISSED_SEEN_KEY = 'scheduleMissedSeen';
const ACTIONS = [
  {id: ACTION.startMowing, Icon: PlayIcon, label: 'Start', main: true},
  {id: ACTION.pause, Icon: StopIcon, label: 'Pause'},
  {id: ACTION.goHome, Icon: HomeIcon, label: 'Go home'},
  {id: ACTION.skipArea, Icon: SkipIcon, label: 'Skip area'},
];


function headline(state: MowerState, docked: boolean, chargeState: string | undefined, area: string | undefined) {
  if (state.emergency) return {title: tr('Emergency stop'), tone: 'error'};
  if (docked) return chargeState === 'Done' ? {title: tr('Charged, in the dock'), tone: 'good'} : {title: tr('Charging in the dock'), tone: 'good'};
  switch (state.current_state) {
    case 'MOWING':
      return {title: area ? tr('Mowing {area}', {area}) : tr('Mowing'), tone: 'live'};
    case 'DOCKING':
      return {title: tr('Heading home'), tone: 'live'};
    case 'UNDOCKING':
      return {title: tr('Leaving the dock'), tone: 'live'};
    case 'PAUSED':
      return {title: tr('Paused'), tone: 'warn'};
    case 'AREA_RECORDING':
      return {title: tr('Recording an area'), tone: 'live'};
    case 'IDLE':
      return {title: tr('Waiting on the lawn'), tone: 'warn'};
    default:
      return {title: state.current_state.toLowerCase().replace(/_/g, ' '), tone: 'neutral'};
  }
}

function BatteryRing({percent, charging}: {percent: number; charging: boolean}) {
  const r = 34;
  const c = 2 * Math.PI * r;
  return (
    <div className={[styles.ring, styles[`ring-${batteryColor(percent)}`]].join(' ')}>
      <svg viewBox="0 0 80 80">
        <circle cx="40" cy="40" r={r} className={styles.ringBg} />
        <circle cx="40" cy="40" r={r} className={styles.ringFill} strokeDasharray={`${(c * percent) / 100} ${c}`} />
      </svg>
      <div>
        <strong>{percent}%</strong>
        {charging && <span>{tr('charging')}</span>}
      </div>
    </div>
  );
}

function LastRun({run}: {run: Run}) {
  const outcome = OUTCOMES[run.outcome];
  const day = new Date(run.start * 1000);
  const today = dayKey(day) === dayKey(new Date());
  return (
    <div className={[styles.lastRun, styles[`run-${outcome.tone}`]].join(' ')}>
      <div>
        <strong>
          {!today && `${dayLabel(day)}, `}
          {clock(run.start)} – {clock(run.end)}
        </strong>
        <span className={styles.dim}>
          {run.areas.join(', ') || tr('no area')}
          {run.bladeSeconds > 0 && ` · ${tr('mowed {time}', {time: duration(run.bladeSeconds)})}`}
        </span>
      </div>
      <span className={styles.badge}>{tr(outcome.label)}</span>
    </div>
  );
}

export default function Home() {
  useLang();
  const link = useMowerState();
  const {state, connected} = link;
  // offline or no fresh state: what's shown is old and commands wouldn't arrive, so the buttons are off
  const live = isLive(link);
  const {hasAction, publishAction} = useMowerActions();
  const {values} = useMowerSensors();
  const position = useMowerPosition() ?? state?.pose;
  const speed = useComputedSpeed(position);
  const track = useMowerTrack();
  const map = useMowerMap();
  const recent = useRecentRuns(state?.current_state);
  const progress = usePlanProgress(state ?? null, map, recent?.today);
  const params = useMowerParams();
  const settings = useSyncExternalStore(settingsStore.subscribe, settingsStore.snapshot, settingsStore.serverSnapshot);
  const weather = useWeather(datumFromParams(params), !!settings.weather);
  const [schedule, setSchedule] = useState<Schedule | null>(() => cachedSchedule() ?? null);
  // the scheduler's log, for a start it left out (rain, battery, ...). again every minute while the page is open
  const [scheduleLog, setScheduleLog] = useState<LogEntry[]>(() => cachedScheduleLog() ?? []);
  useEffect(() => {
    void loadSchedule().then(setSchedule);
    const loadLog = () => void loadScheduleLog().then(setScheduleLog);
    loadLog();
    const timer = setInterval(loadLog, 60000);
    return () => clearInterval(timer);
  }, []);
  // the left out start closed on this device, by its time
  const [missedSeen, setMissedSeen] = useState(() => {
    try {
      return Number(localStorage.getItem(MISSED_SEEN_KEY)) || 0;
    } catch {
      return 0;
    }
  });
  const missedAny = schedule?.enabled ? missedStart(scheduleLog) : null;
  const missed = missedAny && missedAny.t !== missedSeen ? missedAny : null;
  const datum = datumFromParams(params);
  const planned = schedule ? nextStart(schedule, undefined, datum ? sunTimes(datum.lat, datum.lon) : null) : null;
  const [confirmReset, setConfirmReset] = useState(false);
  // seconds until a tapped skip goes out, null when none is waiting
  const [skipLeft, setSkipLeft] = useState<number | null>(null);
  useEffect(() => {
    if (skipLeft === null) return;
    const t = setTimeout(() => {
      if (skipLeft > 1) return setSkipLeft(skipLeft - 1);
      setSkipLeft(null);
      publishAction(ACTION.skipArea);
    }, 1000);
    return () => clearTimeout(t);
  }, [skipLeft, publishAction]);

  const current = state?.current_state ?? '';
  const driving = DRIVING.has(current);
  // a waiting skip is dropped when there's nothing to skip anymore or the connection went
  if (skipLeft !== null && (!live || current !== 'MOWING' || !hasAction(ACTION.skipArea))) setSkipLeft(null);
  const showMap = !!map && (driving || settings.dashboard?.map === 'always');
  const docked = isDocked(state, values['om_v_charge']);
  const battery = state ? Math.round(state.battery_percentage * 100) : 0;
  const chargeState = values['om_charge_state'];
  const charging = docked && chargeState !== 'Done';

  // what the event history knows: since when this state holds and which area is being mowed
  const events = recent?.today ?? [];
  const lastOf = (f: (e: MowerEvent) => boolean) => events.filter(f).pop();
  // only when the newest state change is this state, older mowers don't record every one (a pause, say)
  const lastState = lastOf((e) => e.type === 'STATE');
  const since = lastState?.state === current ? lastState.t : undefined;
  const area = current === 'MOWING' ? lastOf((e) => e.type === 'AREA')?.area_name : undefined;
  const head = state ? headline(state, docked, chargeState, area) : null;

  const acc = state?.pose.pos_accuracy;
  const gpsQ = gpsQuality(acc, numParam(params, PARAM.maxPositionAccuracy));
  const num = (id: string) => (values[id] !== undefined ? Number(values[id]) : undefined);
  const motorTemp = Math.max(...['om_left_esc_temp', 'om_right_esc_temp', 'om_mow_esc_temp'].map((id) => num(id) ?? -Infinity));
  const facts: {label: string; value: string; warn?: boolean}[] = [];
  if (state) {
    facts.push({
      label: 'GPS',
      value:
        gpsQ === 'none' ? (driving ? tr('no fix') : tr('off')) : `${fmt(acc! * 100, 1)} cm · ${tr(GPS_QUALITY_LABEL[gpsQ])}`,
      // only an rtk fix is good while driving
      warn: driving && gpsQ !== 'fix',
    });
    if (driving) facts.push({label: tr('Speed'), value: `${fmt(speed, 2)} m/s`});
    if (progress) {
      const left = progress.secondsLeft !== null ? ` · ${tr('{time} left', {time: duration(progress.secondsLeft)})}` : '';
      facts.push({label: tr('Progress'), value: `${Math.round(progress.fraction * 100)} %${left}`});
    }
    if (charging && num('om_charge_current') !== undefined) facts.push({label: tr('Charging'), value: `${fmt(num('om_charge_current')!, 1)} A`});
    if (num('om_v_battery') !== undefined) facts.push({label: tr('Battery'), value: `${fmt(num('om_v_battery')!, 1)} V`});
    if (motorTemp > -Infinity) facts.push({label: tr('Motors'), value: `${Math.round(motorTemp)} °C`, warn: motorTemp > 70});
    if (state.rain_detected) facts.push({label: tr('Rain'), value: tr('detected'), warn: true});
  }

  const runs = recent?.runs ?? [];
  const mowed = runs.reduce((s, r) => s + r.bladeSeconds, 0);
  const problems = runs.reduce((s, r) => s + r.problems, 0);

  return (
    <div className={styles.page}>
      <main className={[styles.main, showMap ? styles.withMap : ''].join(' ')}>
        <h1 className={styles.brand}>
          <LogoMark size={40} />
          <span>
            <strong>mow</strong>bite
          </span>
          <LoveMower />
        </h1>
        <MowerSwitch className={styles.mowerSwitch} />
        <UpdateBanner />

        {!state && <p className={styles.dim}>{connected ? tr('waiting for the mower…') : tr('connecting…')}</p>}

        {state && head && (
          <section className={[styles.status, live ? styles[`tone-${head.tone}`] : styles.stale].join(' ')}>
            <div className={styles.statusTop}>
              <BatteryRing percent={battery} charging={charging} />
              <div className={styles.headline}>
                <h2>{head.title}</h2>
                {live ? (
                  <span className={styles.dim}>
                    {state.emergency
                      ? tr('Release the mower, then reset the emergency to drive again.')
                      : since
                        ? tr('since {time}', {time: clock(since)})
                        : ''}
                  </span>
                ) : (
                  <span className={styles.offline}>
                    {tr(connected ? 'No data from the mower since {time}' : 'Connection lost, last data at {time}', {
                      time: clock(link.lastAt / 1000, true),
                    })}
                  </span>
                )}
              </div>
            </div>

            <div className={styles.facts}>
              {facts.map((f) => (
                <div key={f.label} className={f.warn ? styles.warn : undefined}>
                  <span>{f.label}</span>
                  <strong>{f.value}</strong>
                </div>
              ))}
            </div>

            <div className={styles.controls}>
              {ACTIONS.map((a) =>
                // paused: the pause button turns into continue. only by the state, the mower also offers continue
                // while mowing until it was paused once
                a.id === ACTION.pause && state.current_state === 'PAUSED' && hasAction(ACTION.resume)
                  ? {...a, id: ACTION.resume, Icon: PlayIcon, label: 'Continue'}
                  : a,
              ).map((a) => (
                <button
                  key={a.id}
                  className={a.main ? styles.main : undefined}
                  // the mower still offers start while the emergency stop is active, it has to be reset first
                  disabled={!live || !hasAction(a.id) || !!state.emergency}
                  onClick={() =>
                    a.id === ACTION.skipArea ? setSkipLeft(skipLeft === null ? SKIP_DELAY : null) : publishAction(a.id)
                  }
                >
                  <a.Icon size={20} />
                  {a.id === ACTION.skipArea && skipLeft !== null ? tr('Undo ({n} s)', {n: skipLeft}) : tr(a.label)}
                </button>
              ))}
              {live &&
                hasAction(ACTION.resetJob) &&
                !state.emergency &&
                (confirmReset ? (
                  <div className={styles.resetJob}>
                    <span>{tr('Drop the interrupted job? The next start mows from the beginning.')}</span>
                    <button
                      className={styles.reset}
                      onClick={() => {
                        publishAction(ACTION.resetJob);
                        setConfirmReset(false);
                      }}
                    >
                      {tr('Drop it')}
                    </button>
                    <button onClick={() => setConfirmReset(false)}>{tr('Cancel')}</button>
                  </div>
                ) : (
                  <button className={styles.resetJobButton} onClick={() => setConfirmReset(true)}>
                    {tr('Drop the interrupted job')}
                  </button>
                ))}
              {!!state.emergency && (
                <button className={styles.reset} disabled={!live} onClick={() => publishAction(ACTION_RESET_EMERGENCY)}>
                  <WarningIcon size={20} />
                  {tr('Reset emergency')}
                </button>
              )}
            </div>
          </section>
        )}

        {showMap && map && (
          <section className={styles.map}>
            <MapView
              map={map}
              mower={position}
              emergency={!!state?.emergency}
              track={track}
              progress={progress ?? undefined}
              follow={driving}
              zoomable
              datum={datumFromParams(params)}
            />
          </section>
        )}

        {recent && (
          <section className={styles.today}>
            <div className={styles.todayHead}>
              <h2>{tr('Today')}</h2>
              <Link href="/activity">{tr('Activity')}</Link>
            </div>
            {weather && (
              <div className={styles.weather}>
                <WeatherIcon code={weather.code} day={weather.day} />
                <div>
                  <strong>
                    {fmt(weather.temp)}°{' '}
                    <span className={styles.dim}>{tr(WEATHER_LABELS[weatherKind(weather.code)])}</span>
                  </strong>
                  <span className={styles.dim}>
                    {tr('max. {max}°, min. {min}°', {max: fmt(weather.max), min: fmt(weather.min)})}
                  </span>
                </div>
                {(weather.raining || weather.rainAt) && (
                  <span className={styles.rainHint}>
                    {weather.raining
                      ? tr('Raining')
                      : weather.rainSoon
                        ? tr('Rain soon')
                        : tr('Rain from about {time}', {time: clock(weather.rainAt!)})}
                  </span>
                )}
                <a className={styles.credit} href="https://open-meteo.com" target="_blank" rel="noreferrer">
                  Open-Meteo
                </a>
              </div>
            )}
            <div className={styles.todayNumbers}>
              <div>
                <strong>{mowed ? duration(mowed) : '0 min'}</strong>
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
            </div>
            {missed && (
              <div className={styles.missed}>
                <WarningIcon size={16} />
                <Link href="/schedule">
                  <strong>
                    {dayKey(new Date(missed.t * 1000)) === dayKey(new Date())
                      ? tr('Scheduled start at {time}', {time: clock(missed.t)})
                      : tr('Scheduled start yesterday at {time}', {time: clock(missed.t)})}
                  </strong>
                  <span>{tr(SCHEDULE_LOG_TEXT[missed.what] ?? missed.what, {detail: missed.detail ?? ''})}</span>
                </Link>
                <button
                  title={tr('Close')}
                  aria-label={tr('Close')}
                  onClick={() => {
                    setMissedSeen(missed.t);
                    try {
                      localStorage.setItem(MISSED_SEEN_KEY, String(missed.t));
                    } catch {}
                  }}
                >
                  ×
                </button>
              </div>
            )}
            {planned && (
              <Link href="/schedule" className={styles.nextRun}>
                {tr('Next start: {when}', {when: `${dayLabel(planned)}, ${clock(planned.getTime() / 1000)}`})}
              </Link>
            )}
            {recent.last && (
              <>
                <span className={styles.label}>{tr('Last run')}</span>
                <LastRun run={recent.last} />
              </>
            )}
          </section>
        )}
      </main>
    </div>
  );
}
