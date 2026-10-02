'use client';

import {getMqttClient, onTopic} from '@/lib/mqttClient';
import {callRpc} from '@/lib/rpc';
import {useSyncExternalStore} from 'react';
import {RPC, TOPIC} from '@/lib/openmower';

export interface Point {
  x: number;
  y: number;
  // blades running at that point, false = just driving
  b?: boolean;
}

// min distance between trail points, and how far a point may be off the line through its neighbours to get dropped
// (straight stripes need only their ends)
const MIN_DISTANCE = 0.03;
const STRAIGHT = 0.01;
const MAX_POINTS = 30000;
// the trail is kept in pieces of this many points. finished pieces never change, so each new position only
// touches the last one and the map only redraws that, however long the job is
const CHUNK = 400;
// at most this often the map gets told about new points, positions come in several times a second
const NOTIFY_MS = 200;

// the pieces in order, the last one is still growing. each piece starts with the last point of the one before,
// so they join up when drawn
export type TrackChunks = readonly Point[][];
const EMPTY: TrackChunks = [];
let track: TrackChunks = EMPTY;
let started = false;
let jobId: string | null = null;
let blades: boolean | undefined;
// what came in live while the recorded track was loading
let whileLoading: Point[] = [];

// whether b lies on the line from a to c
function straight(a: Point, b: Point, c: Point) {
  if (a.b !== b.b || b.b !== c.b) return false;
  const dx = c.x - a.x;
  const dy = c.y - a.y;
  const len = Math.hypot(dx, dy);
  if (len === 0) return true;
  // and between them, not a turn back
  const t = ((b.x - a.x) * dx + (b.y - a.y) * dy) / (len * len);
  return t >= 0 && t <= 1 && Math.abs(dy * (b.x - a.x) - dx * (b.y - a.y)) / len < STRAIGHT;
}

// appends p to out, in place
function add(out: Point[], p: Point) {
  const last = out[out.length - 1];
  if (last && last.b === p.b && Math.hypot(p.x - last.x, p.y - last.y) < MIN_DISTANCE) return;
  if (out.length >= 2 && straight(out[out.length - 2], last, p)) out[out.length - 1] = p;
  else out.push(p);
}

export function thin(points: Point[]): Point[] {
  const out: Point[] = [];
  for (const p of points) add(out, p);
  return out;
}

export function chunked(points: Point[]): TrackChunks {
  if (!points.length) return EMPTY;
  const out: Point[][] = [];
  for (let i = 0; ; i += CHUNK - 1) {
    out.push(points.slice(i, i + CHUNK));
    if (i + CHUNK >= points.length) break;
  }
  return out;
}

export function trackPoints(chunks: TrackChunks) {
  return chunks.reduce((n, c, i) => n + c.length - (i ? 1 : 0), 0);
}

// the trail with p added to the growing piece, null when it doesn't change anything
export function appendPoint(chunks: TrackChunks, p: Point, maxPoints = MAX_POINTS): TrackChunks | null {
  const tail = chunks[chunks.length - 1];
  if (!tail) return [[p]];
  const next = tail.slice();
  const last = next[next.length - 1];
  const before = next.length;
  add(next, p);
  if (next.length === before && next[next.length - 1] === last) return null;
  const out = chunks.slice(0, -1);
  if (next.length > CHUNK) out.push(next.slice(0, CHUNK), next.slice(CHUNK - 1));
  else out.push(next);
  // the oldest pieces go first once there are too many points
  let total = trackPoints(out);
  while (out.length > 1 && total > maxPoints) total -= out.shift()!.length - 1;
  return out;
}

function append(p: Point): boolean {
  const next = appendPoint(track, p);
  if (!next) return false;
  track = next;
  return true;
}

let notifyTimer: ReturnType<typeof setTimeout> | null = null;
function notifySoon() {
  if (notifyTimer) return;
  notifyTimer = setTimeout(() => {
    notifyTimer = null;
    listeners.forEach((l) => l());
  }, NOTIFY_MS);
}

// what the mower recorded of this job so far, so a reload doesn't start with an empty trail. again after the
// connection was gone (phone asleep, app in the background), the live trail misses that part
let loading = false;

async function recordedTrack(id: string): Promise<Point[]> {
  const h = await callRpc<{
    segments?: {points: [number, number][]; attributes?: {blades?: boolean}}[];
    buffer?: [number, number][];
  }>(RPC.jobTrack, {job_id: id}, 20000);
  // the buffer is the segment still being written, so it has the current blade state
  return [
    ...(h.segments ?? []).flatMap((s) => s.points.map(([x, y]) => ({x, y, b: s.attributes?.blades}))),
    ...(h.buffer ?? []).map(([x, y]) => ({x, y, b: blades})),
  ];
}

// cleared by hand: the job and how many of its recorded points were there then, only later ones are shown.
// per device, the recording on the mower stays as it is
const CLEARED_KEY = 'trackCleared';
function clearedUpTo(id: string): number {
  try {
    const c = JSON.parse(localStorage.getItem(CLEARED_KEY) ?? 'null');
    return c?.job === id && typeof c.points === 'number' ? c.points : 0;
  } catch {
    return 0;
  }
}

async function seed(id: string) {
  if (loading) return;
  loading = true;
  whileLoading = [];
  try {
    const recorded = (await recordedTrack(id)).slice(clearedUpTo(id));
    if (jobId !== id) return;
    // the recorded track replaces what was here, only what came in live while it loaded goes on top
    track = chunked(thin([...recorded, ...whileLoading]).slice(-MAX_POINTS));
    listeners.forEach((l) => l());
  } catch {
    // no history on this mower, just keep the live trail
  } finally {
    loading = false;
  }
}
const listeners = new Set<() => void>();

// recorded straight from mqtt so it keeps going no matter which tab is open
function start() {
  if (started) return;
  started = true;
  const c = getMqttClient();
  // position/json is much more frequent, robot_state is the fallback for setups without it
  let fast = false;
  // after a reconnect the trail of the time without connection comes from the recorded track
  c.on('connect', () => jobId && void seed(jobId));
  // a phone keeps the connection a while in the background but gets no messages, so also when the page is back
  document.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'visible' && jobId) void seed(jobId);
  });
  const onPose = (payload: Buffer, name: string) => {
    if (name === TOPIC.position) fast = true;
    else if (name !== TOPIC.robotState || fast) return;
    let pose: Point | undefined;
    try {
      const msg = JSON.parse(payload.toString());
      pose = name === TOPIC.position ? msg : msg.pose;
      const id = name === TOPIC.position ? msg.attributes?.job_id : undefined;
      if (name === TOPIC.position) blades = msg.attributes?.blades;
      if (id && id !== jobId) {
        // new job, new trail
        const first = jobId === null;
        jobId = id;
        if (!first) track = EMPTY;
        void seed(id);
      }
    } catch {
      return;
    }
    if (!pose) return;
    if (loading) whileLoading.push({x: pose.x, y: pose.y, b: blades});
    if (append({x: pose.x, y: pose.y, b: blades})) notifySoon();
  };
  onTopic(TOPIC.robotState, onPose);
  onTopic(TOPIC.position, onPose);
}

function subscribe(listener: () => void) {
  start();
  listeners.add(listener);
  return () => listeners.delete(listener);
}

// empties the live trail, it starts again from the mower's position now
export async function clearTrack() {
  const id = jobId;
  if (id) {
    try {
      const points = (await recordedTrack(id)).length;
      localStorage.setItem(CLEARED_KEY, JSON.stringify({job: id, points}));
    } catch {}
  }
  track = EMPTY;
  whileLoading = [];
  listeners.forEach((l) => l());
}

export function useMowerTrack(): TrackChunks {
  return useSyncExternalStore(
    subscribe,
    () => track,
    () => EMPTY,
  );
}
