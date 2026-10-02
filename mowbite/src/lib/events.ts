import {tr} from './i18n';
import {LOGIC_SOURCE} from './openmower';

// events from the mower's event history (events.history rpc), grouped into runs for the activity page

export interface MowerEvent {
  id: string;
  type: string;
  t: number; // unix seconds
  job_id?: string;
  session_id?: string;
  x?: number;
  y?: number;
  state?: string;
  available?: boolean;
  enabled?: boolean;
  area_id?: string;
  area_name?: string;
  reason?: string;
  attempts?: number;
  emergency?: boolean;
}

export type Severity = 'error' | 'warning' | 'info';

const STATES: Record<string, string> = {
  IDLE: 'Idle',
  MOWING: 'Mowing',
  DOCKING: 'Heading home',
  UNDOCKING: 'Leaving the dock',
  PAUSED: 'Paused',
  AREA_RECORDING: 'Recording an area',
};

const REASONS: Record<string, string> = {
  path_failed: "couldn't follow the path",
  approach_failed: 'approach failed',
  dock_failed: "no contact with the charger",
  no_gps: 'no GPS fix',
};

const reason = (r?: string) => (r ? (REASONS[r] ? tr(REASONS[r]) : r.replace(/_/g, ' ').toLowerCase()) : '');

// state is what the mower was doing when the event happened, gps is switched off on purpose outside
// of mowing, so losing it only counts as a problem while mowing
export function describe(e: MowerEvent, state?: string): {text: string; severity: Severity} {
  const info = (text: string) => ({text, severity: 'info' as Severity});
  switch (e.type) {
    case 'STATE':
      return info(STATES[e.state ?? ''] ? tr(STATES[e.state ?? '']) : (e.state ?? tr('State changed')));
    case 'GPS':
      if (e.available) return info(tr('GPS fix'));
      return state === 'MOWING' ? {text: tr('GPS lost'), severity: 'warning'} : info(tr('GPS off'));
    case 'BLADES':
      return info(e.enabled ? tr('Blades on') : tr('Blades off'));
    case 'AREA':
      return info(tr('Mowing "{area}"', {area: e.area_name || tr('unnamed')}));
    case 'AREA_SKIPPED':
      // only happens when someone presses skip
      return info(tr('Area skipped'));
    case 'UNDOCKED':
      return info(tr('Left the dock'));
    case 'UNDOCKING_FAILED':
      return {
        text: e.reason ? tr('Leaving the dock failed ({reason})', {reason: reason(e.reason)}) : tr('Leaving the dock failed'),
        severity: 'error',
      };
    case 'DOCKING':
      return info(e.reason ? tr('Heading home: {reason}', {reason: reason(e.reason)}) : tr('Heading home'));
    case 'DOCKING_RETRY':
      return {text: tr('Docking retry {n} ({reason})', {n: e.attempts ?? '', reason: reason(e.reason)}).replace('  ', ' '), severity: 'warning'};
    case 'DOCKING_FAILED':
      return {text: tr('Docking failed after {n} tries ({reason})', {n: e.attempts ?? '?', reason: reason(e.reason)}), severity: 'error'};
    case 'DOCKED':
      return info(tr('Docked'));
    case 'JOB_COMPLETE':
      return info(tr('All areas done'));
    case 'JOB_RESET':
      return info(tr('Interrupted job dropped, the next start begins from the start'));
    case 'EMERGENCY':
      return e.emergency
        ? {
            text: e.reason && e.reason !== '0' ? tr('Emergency stop (code {code})', {code: e.reason}) : tr('Emergency stop'),
            severity: 'error',
          }
        : info(tr('Emergency cleared'));
    case 'NAVIGATION_ERROR':
      return {text: tr('Navigation error'), severity: 'error'};
    case 'BOOTED':
      return info(tr('Mower started'));
    case 'SHUTDOWN':
      return info(tr('Shut down'));
    default:
      return info(e.type.replace(/_/g, ' ').toLowerCase());
  }
}

// what the mower's code does when it logs these, the event itself carries nothing more
// gpsTimeout: the mower's mower_logic/gps_timeout in seconds, if known
export function explain(e: MowerEvent, state?: string, gpsTimeout?: number): string | undefined {
  switch (e.type) {
    case 'UNDOCKING_FAILED':
      if (e.reason === 'no_gps')
        return tr(
          'It got out of the dock but had no RTK fix in time and stopped. Check the NTRIP corrections and whether the GPS antenna has a clear sky view there.',
        );
      return tr(
        "It couldn't drive the short way out of the dock (backwards, then turning) and gave up. Usual causes: the mower's idea of its heading is off while it stands in the dock, it's blocked (wheels, grass, bumper), or it's still in emergency mode.",
      );
    case 'DOCKING_RETRY':
    case 'DOCKING_FAILED':
      return e.reason === 'dock_failed'
        ? tr(
            "It was in front of the dock but didn't get charging contact, so it backed out to try again. Check the contacts and whether the dock position on the map is still right.",
          )
        : tr(
            "It couldn't reach the point in front of the dock. Usually the GPS fix was bad or the way there is blocked or outside the navigation area.",
          );
    case 'NAVIGATION_ERROR':
      return tr(
        "It couldn't follow the mowing path and the recovery didn't help, so it paused. Often something is in the way, or the path runs too close to an obstacle or the edge.",
      );
    case 'GPS':
      if (e.available || state !== 'MOWING') return undefined;
      return gpsTimeout !== undefined
        ? tr(
            'The RTK fix had been gone for {n} s, that long it keeps driving on wheel odometry (mower_logic/gps_timeout). Then it stops with the blade off and waits until the fix is back.',
            {n: Math.round(gpsTimeout)},
          )
        : tr(
            'The RTK fix had been gone for longer than the mower allows (mower_logic/gps_timeout), so it stopped with the blade off and waits until the fix is back.',
          );
    case 'EMERGENCY':
      return e.emergency ? tr('Stop button, lift, tilt or a bumper. It has to be released and reset before it drives again.') : undefined;
  }
}

// where open_mower_ros writes the event and the line it logs at the same moment
export function eventSource(e: MowerEvent): {file: string; url: string; log?: string} | undefined {
  const src = (file: string, log?: string) => ({file, url: LOGIC_SOURCE + file, log});
  switch (e.type) {
    case 'UNDOCKING_FAILED':
      return src('behaviors/UndockingBehavior.cpp', e.reason === 'no_gps' ? 'Could not get GPS.' : 'Error during undock');
    case 'DOCKING_RETRY':
      return src('behaviors/DockingBehavior.cpp', e.reason === 'dock_failed' ? 'Error during docking.' : 'Error during docking approach.');
    case 'DOCKING_FAILED':
      return src('behaviors/DockingBehavior.cpp', 'Giving up on docking');
    case 'NAVIGATION_ERROR':
      return src('behaviors/MowingBehavior.cpp', 'MowingBehavior: (MOW) PAUSED due to MBF Error at …');
    case 'EMERGENCY':
    case 'GPS':
      return src('mower_logic.cpp');
  }
}

// the line as it is in the file: id, t, type, the event's own fields, then position and ids
export function rawLine(e: MowerEvent): string {
  const {id, t, type, x, y, job_id, session_id, ...rest} = e;
  return JSON.stringify({id, t, type, ...rest, x, y, job_id, session_id});
}

export type Outcome = 'done' | 'paused' | 'undock_failed' | 'dock_failed' | 'emergency' | 'returned' | 'running';

export interface Run {
  start: number;
  end: number;
  events: MowerEvent[];
  areas: string[];
  bladeSeconds: number;
  problems: number;
  outcome: Outcome;
  jobId?: string;
}

export type Entry = {kind: 'run'; run: Run} | {kind: 'event'; event: MowerEvent};

// each event with the state the mower was in at that moment
export function withState(events: MowerEvent[]): {event: MowerEvent; state?: string}[] {
  let state: string | undefined;
  return events.map((event) => {
    const before = state;
    if (event.type === 'STATE') state = event.state;
    return {event, state: before};
  });
}

// A run starts when the mower leaves the dock and ends when it's docked again (or idle/off). Everything
// in between belongs to it, the rest (boot, shutdown, gps while parked) stays a loose event.
export function groupRuns(events: MowerEvent[], live: boolean): Entry[] {
  const sorted = [...events].sort((a, b) => a.t - b.t);
  const out: Entry[] = [];
  let run: MowerEvent[] | null = null;

  const close = (finished: boolean) => {
    if (run?.length) out.push({kind: 'run', run: summarize(run, finished || !live)});
    run = null;
  };

  for (const e of sorted) {
    const starts = e.type === 'STATE' && (e.state === 'UNDOCKING' || e.state === 'MOWING');
    if (!run && starts) run = [];
    if (!run) {
      out.push({kind: 'event', event: e});
      continue;
    }
    run.push(e);
    const ends =
      e.type === 'DOCKED' ||
      e.type === 'SHUTDOWN' ||
      (e.type === 'STATE' && e.state === 'IDLE' && run.some((r) => r.type === 'UNDOCKING_FAILED'));
    if (ends) close(true);
  }
  close(false);
  return out;
}

function summarize(events: MowerEvent[], finished: boolean): Run {
  const areas: string[] = [];
  let bladeSeconds = 0;
  let bladesSince: number | null = null;
  let problems = 0;
  for (const {event: e, state} of withState(events)) {
    if (e.type === 'AREA' && e.area_name && !areas.includes(e.area_name)) areas.push(e.area_name);
    if (e.type === 'BLADES') {
      if (e.enabled && bladesSince === null) bladesSince = e.t;
      if (!e.enabled && bladesSince !== null) {
        bladeSeconds += e.t - bladesSince;
        bladesSince = null;
      }
    }
    if (describe(e, state).severity !== 'info') problems++;
  }
  const end = events[events.length - 1].t;
  if (bladesSince !== null) bladeSeconds += end - bladesSince;

  const has = (type: string) => events.some((e) => e.type === type);
  let outcome: Outcome = 'returned';
  if (!finished) outcome = 'running';
  else if (has('JOB_COMPLETE')) outcome = 'done';
  else if (events.some((e) => e.type === 'EMERGENCY' && e.emergency)) outcome = 'emergency';
  else if (has('DOCKING_FAILED')) outcome = 'dock_failed';
  else if (has('UNDOCKING_FAILED') && !has('UNDOCKED')) outcome = 'undock_failed';
  else if (events.some((e) => e.type === 'DOCKING' && e.reason === 'Manual pause')) outcome = 'paused';

  return {
    start: events[0].t,
    end,
    events,
    areas,
    bladeSeconds,
    problems,
    outcome,
    jobId: events.find((e) => e.job_id)?.job_id,
  };
}

export const OUTCOMES: Record<Outcome, {label: string; tone: 'good' | 'neutral' | 'bad' | 'live'}> = {
  done: {label: 'Finished', tone: 'good'},
  paused: {label: 'Paused by you', tone: 'neutral'},
  returned: {label: 'Back in the dock', tone: 'neutral'},
  undock_failed: {label: "Couldn't leave the dock", tone: 'bad'},
  dock_failed: {label: 'Docking failed', tone: 'bad'},
  emergency: {label: 'Emergency stop', tone: 'bad'},
  running: {label: 'Running', tone: 'live'},
};
