'use client';

import {datumFromParams, useMowerParams} from '@/hooks/useMowerParams';
import {isNightTime} from '@/lib/schedule';
import {settingsStore} from '@/lib/settings';
import {seasonOf, sunTimes} from '@/lib/sun';
import {usePathname} from 'next/navigation';
import {useEffect, useRef, useState, useSyncExternalStore} from 'react';
import LogoMark from './Logo';
import {leg, pushMower, stepDur} from './mapIcons';
import styles from './Grass.module.css';

const SPEED = 260; // px/s
const REGROW_MS = 2500;
// the hedgehog comes out, yawns and looks around before the mower starts
const PRE_MS = 2200;
// the hop over the hedgehog starts this long before the mower gets there
const HOP_LEAD = 360;
// after the eater is over it (ms): it sighs, is cross with the eater, yawns, and the leaves fly back onto it
const SIGH_AFTER = 800;
const CROSS_AFTER = 1900;
const YAWN_AFTER = 3100;
const BURY_AFTER = 3800;
// till the last leaf is back and it's asleep under them
const BURIED_MS = 1700;

// autumn nights now and then a hedgehog comes in from the right, stops to sniff, and crawls under the heap of leaves.
// looked at every half minute: about every 5 minutes in the first hour and a half after
// sunset, every 10 later, and not again within half an hour on this device
const WALK_CHECK_MS = 30000;
const WALK_SPEED = 42; // px/s
const SNIFF_MS = 1800;
const HIDE_MS = 1000;
const WALK_GAP_MS = 30 * 60000;
const WALK_KEY = 'hedgehogWalk';
// HH:MM, local
const clock = (d: Date) => d.toTimeString().slice(0, 5);

// autumn now and then a gust of wind: the leaves in the air are blown aside, the grass bends, the ones on the lawn
// lift a little and one flies along. rarely, every one to three and a half minutes
const GUST_MS = 3500;
const GUST_MIN_MS = 60000;
const GUST_MAX_MS = 210000;

// winter: two quick taps on the snowman and father christmas mows the snow off the lawn, then it snows again
const SANTA_SPEED = 150; // px/s
// the snowman looks out for him first
const SANTA_PRE_MS = 1200;
// how fast his legs and the wheels go, in m/s like on the map
const SANTA_STEP = 0.8;

// a hedgehog that sees the lawn eater coming, trembles and curls up, and gets hopped over. side view, facing left
function Hedgehog() {
  return (
    <svg width="52" height="35" viewBox="0 0 50 34">
      <g className={styles.head}>
        <g className={styles.body}>
        <ellipse cx="27" cy="30" rx="15" ry="4" fill="#c49a6c" />
        <path d="M22 17.5 C14 17.5 8.5 21.5 2.5 26 C8.5 30.5 14 32 22 31.5 Z" fill="#d8b48a" />
        <g>
          <path d="M12.5 31.0 L16.3 29.2 L13.0 26.5 L17.2 25.6 L14.6 22.2 L18.8 22.4 L17.2 18.4 L21.2 19.6 L20.5 15.4 L24.1 17.5 L24.4 13.2 L27.5 16.2 L28.8 12.1 L31.0 15.8 L33.2 12.1 L34.5 16.2 L37.6 13.2 L37.9 17.5 L41.5 15.4 L40.8 19.6 L44.8 18.4 L43.2 22.4 L47.4 22.2 L44.8 25.6 L49.0 26.5 L45.7 29.2 L49.5 31.0 Z" fill="#5b3d22" />
          <path d="M17.0 31.0 L19.2 29.1 L17.7 26.5 L20.4 25.4 L19.7 22.5 L22.6 22.3 L22.8 19.3 L25.6 20.0 L26.7 17.2 L29.1 18.8 L31.0 16.5 L32.9 18.8 L35.3 17.2 L36.4 20.0 L39.2 19.3 L39.4 22.3 L42.3 22.5 L41.6 25.4 L44.3 26.5 L42.8 29.1 L45.0 31.0 Z" fill="#7a5433" />
        </g>
        {/* cross with the eater */}
        <path
          className={styles.anger}
          d="M6 12 Q7.5 12 7.5 10.5 M10 10.5 Q10 12 11.5 12 M11.5 14.5 Q10 14.5 10 16 M7.5 16 Q7.5 14.5 6 14.5"
          stroke="#e53935"
          strokeWidth="1.1"
          strokeLinecap="round"
          fill="none"
        />
        {/* a leaf that gets stuck on the spines */}
        <g className={styles.backLeaf}>
          <path d="M30 12.5 C33 10.5 37 11.5 38 14.5 C35 16 31.5 15.5 30 12.5 Z" fill="#d9822b" />
          <path d="M30.5 12.8 L37.5 14.3" stroke="#8d5a2b" strokeWidth="0.5" />
        </g>
        <ellipse className={styles.foot} cx="18" cy="32.6" rx="2.6" ry="1.4" fill="#4a3220" />
        <ellipse className={styles.foot} cx="33" cy="32.8" rx="2.6" ry="1.3" fill="#3a2718" />
        <circle cx="16.5" cy="19" r="1.9" fill="#c49a6c" />
        <circle className={styles.nose} cx="3.2" cy="26" r="1.7" fill="#1b1b1b" />
        <g className={styles.eyes}>
          <circle cx="10.5" cy="23" r="2.4" fill="#fff" />
          <g className={styles.pupils}>
            <circle cx="10.3" cy="23.2" r="1.2" fill="#1b1b1b" />
          </g>
          <g className={styles.lids}>
            <rect x="7.9" y="20.4" width="5.2" height="5.2" rx="2.5" fill="#d8b48a" />
          </g>
        </g>
        <path className={styles.smile} d="M5.6 28.4 Q7.6 29.8 9.6 28.6" stroke="#7a4e2a" strokeWidth="0.9" fill="none" strokeLinecap="round" />
        <ellipse className={styles.gasp} cx="7.6" cy="29" rx="0.9" ry="1.2" fill="#7a4e2a" />
        </g>
        {/* curled up */}
        <g className={styles.ball}>
          <path d="M38.5 22.5 L35.5 24.4 L37.4 27.5 L33.8 27.9 L34.2 31.5 L30.8 30.4 L29.6 33.7 L27.0 31.2 L24.4 33.7 L23.2 30.4 L19.8 31.5 L20.2 27.9 L16.6 27.5 L18.5 24.4 L15.5 22.5 L18.5 20.6 L16.6 17.5 L20.2 17.1 L19.8 13.5 L23.2 14.6 L24.4 11.3 L27.0 13.8 L29.6 11.3 L30.8 14.6 L34.2 13.5 L33.8 17.1 L37.4 17.5 L35.5 20.6 Z" fill="#5b3d22" />
          <path d="M35.2 23.3 L33.2 25.0 L33.4 27.6 L30.9 28.0 L29.6 30.3 L27.3 29.2 L25.0 30.5 L23.6 28.3 L21.0 28.1 L21.0 25.6 L18.9 24.0 L20.3 21.8 L19.4 19.4 L21.7 18.3 L22.3 15.8 L24.8 16.1 L26.6 14.3 L28.6 16.0 L31.1 15.4 L31.9 17.9 L34.3 18.8 L33.6 21.3 Z" fill="#7a5433" />
        </g>
      </g>
    </svg>
  );
}

// fairy lights along the handle and over the hood of the mower, each on its own beat
const LIGHTS = [
  [0.007, -0.124],
  [0.12, 0.02],
  [0.23, 0.165],
  [0.345, 0.31],
  [0.42, 0.31],
  [0.52, 0.27],
  [0.64, 0.255],
  [0.76, 0.265],
  [0.88, 0.31],
  [0.97, 0.39],
].map(([x, y], i) => ({x, y, color: ['#ff5252', '#ffd740', '#40c4ff', '#69f0ae', '#ff9100'][i % 5], begin: (i % 3) * 0.4}));

// snow thrown up ahead of the mower
const SNOW_SPRAY = [0, 0.15, 0.3, 0.45, 0.6].map((d, i) => (
  <circle key={i} cx={1.04} cy={0.62} r={0.035 + (i % 2) * 0.015} fill="#fff" opacity={0}>
    <animate attributeName="cx" values={`1.04;${1.2 + i * 0.04};${1.32 + i * 0.05}`} dur="0.75s" begin={`${d}s`} repeatCount="indefinite" />
    <animate attributeName="cy" values="0.62;0.05;0.45" dur="0.75s" begin={`${d}s`} repeatCount="indefinite" />
    <animate attributeName="opacity" values="0;1;0" dur="0.75s" begin={`${d}s`} repeatCount="indefinite" />
  </circle>
));

// father christmas pushing the gardener's mower (from the map symbols) through the snow, facing right. the front of
// the mower is 51 px into the drawing
function Santa() {
  return (
    <svg width="64" height="57" viewBox="-0.8 -1.15 2.3 2.05" overflow="visible">
      {pushMower(SANTA_STEP, '#2e7d32')}
      <polyline points={LIGHTS.map((l) => `${l.x},${l.y}`).join(' ')} fill="none" stroke="#1b5e20" strokeWidth={0.02} />
      {LIGHTS.map((l, i) => (
        <g key={i}>
          <animate attributeName="opacity" values="1;0.35;1" dur="1.2s" begin={`${l.begin}s`} repeatCount="indefinite" />
          <circle cx={l.x} cy={l.y} r={0.07} fill={l.color} opacity={0.35} />
          <circle cx={l.x} cy={l.y} r={0.032} fill={l.color} />
        </g>
      ))}
      {SNOW_SPRAY}
      <g transform="translate(-0.45 0.12)">
        {leg(22, SANTA_STEP, '#c62828', '#1b1b1b')}
        {leg(-22, SANTA_STEP, '#c62828', '#1b1b1b')}
      </g>
      <g>
        <animateTransform
          attributeName="transform"
          type="translate"
          values="0 0;0 -0.035;0 0"
          dur={`${(stepDur(SANTA_STEP) / 2).toFixed(2)}s`}
          repeatCount="indefinite"
        />
        {/* red coat with white fur at the hem, a black belt with a golden buckle */}
        <path d="M-0.64,-0.5 Q-0.45,-0.62 -0.28,-0.5 Q-0.14,-0.18 -0.22,0.16 L-0.66,0.16 Z" fill="#c62828" />
        <path d="M-0.68,0.1 L-0.2,0.1 Q-0.18,0.16 -0.2,0.22 L-0.68,0.22 Q-0.7,0.16 -0.68,0.1 Z" fill="#fafafa" />
        <path d="M-0.66,-0.08 L-0.17,-0.08 L-0.17,0 L-0.66,0 Z" fill="#1b1b1b" />
        <rect x={-0.34} y={-0.1} width={0.1} height={0.12} rx={0.015} fill="none" stroke="#ffca28" strokeWidth={0.025} />
        {/* the arm on the handle, white fur at the cuff, a black glove */}
        <path d="M-0.42,-0.46 L-0.34,-0.52 L-0.04,-0.18 L-0.1,-0.1 Z" fill="#c62828" />
        <path d="M-0.13,-0.24 L-0.07,-0.3 L-0.01,-0.22 L-0.07,-0.15 Z" fill="#fafafa" />
        <circle cx={-0.06} cy={-0.14} r={0.065} fill="#1b1b1b" />
        {/* face, white beard, red nose, the cap with its bobble hanging back */}
        <circle cx={-0.43} cy={-0.72} r={0.17} fill="#f1c27d" />
        <circle cx={-0.34} cy={-0.77} r={0.02} fill="#3e2723" />
        <path
          d="M-0.6,-0.72 Q-0.62,-0.42 -0.42,-0.36 Q-0.22,-0.38 -0.24,-0.66 Q-0.34,-0.6 -0.44,-0.62 Q-0.54,-0.62 -0.6,-0.72 Z"
          fill="#fafafa"
        />
        <circle cx={-0.27} cy={-0.71} r={0.035} fill="#ef5350" />
        <path d="M-0.6,-0.83 C-0.6,-1.02 -0.44,-1.1 -0.32,-1.02 C-0.26,-0.96 -0.25,-0.9 -0.26,-0.83 Z" fill="#c62828" />
        <path d="M-0.44,-1.06 Q-0.66,-1.08 -0.74,-0.88 L-0.67,-0.86 Q-0.6,-1 -0.42,-0.98 Z" fill="#c62828" />
        <circle cx={-0.72} cy={-0.84} r={0.06} fill="#fafafa" />
        <path d="M-0.63,-0.84 Q-0.43,-0.9 -0.23,-0.84 L-0.23,-0.76 Q-0.43,-0.82 -0.63,-0.76 Z" fill="#fafafa" />
      </g>
    </svg>
  );
}

// stands in the snow in the middle, looks out for father christmas and waves at him
function Snowman() {
  return (
    <svg width="34" height="46" viewBox="0 0 34 46">
      <g fill="#f4f8fb" stroke="#cfd8dc" strokeWidth="0.6">
        <circle cx="17" cy="35.5" r="10" />
        <circle cx="17" cy="21" r="7.5" />
        <circle cx="17" cy="9.5" r="5.5" />
      </g>
      <path d="M10.5 20.5 L3 15 M5 16.4 L3.6 13.6" stroke="#6d4c41" strokeWidth="1.2" strokeLinecap="round" fill="none" />
      <path
        className={styles.smArm}
        d="M23.5 20.5 L31 15 M29 16.4 L31.5 14.6"
        stroke="#6d4c41"
        strokeWidth="1.2"
        strokeLinecap="round"
        fill="none"
      />
      <path d="M11.4 14.2 Q17 16.4 22.6 14.2 L22.6 16.6 Q17 18.6 11.4 16.6 Z" fill="#c62828" />
      <path d="M18.5 16 L21 22 L23 21.2 L20.8 15.6 Z" fill="#c62828" />
      <circle cx="17" cy="20.5" r="0.75" fill="#263238" />
      <circle cx="17" cy="24" r="0.75" fill="#263238" />
      <rect x="12.6" y="0.6" width="8.8" height="4.6" rx="0.6" fill="#263238" />
      <rect x="10.6" y="4.6" width="12.8" height="1.4" rx="0.7" fill="#263238" />
      <g className={styles.smEyes}>
        <circle cx="15.2" cy="8.8" r="0.8" fill="#263238" />
        <circle cx="18.8" cy="8.8" r="0.8" fill="#263238" />
      </g>
      <path d="M17 10.6 L11.2 11.6 L17 12.2 Z" fill="#ef6c00" />
    </svg>
  );
}

// snow drifting down behind the page in winter, optional like the leaves. some are already on their way
const FLAKES = Array.from({length: 26}, (_, i) => ({
  left: (i * 37 + 5) % 100,
  size: 3 + (i % 4),
  dur: 11 + ((i * 7) % 9),
  delay: (i * 1.7) % 14,
  sway: 3 + (i % 3),
}));

function FallingSnow() {
  return (
    <div className={styles.falling} aria-hidden="true">
      {FLAKES.map((f, i) => (
        <div
          key={i}
          className={styles.snowFall}
          style={{left: `${f.left}%`, animationDuration: `${f.dur}s`, animationDelay: `-${f.delay}s`}}
        >
          <div className={styles.snowSway} style={{animationDuration: `${f.sway}s`}}>
            <div className={styles.flake} style={{width: f.size, height: f.size}} />
          </div>
        </div>
      ))}
    </div>
  );
}

// the season doesn't change while the page is open, near enough
const noChange = () => () => {};
const LEAF_COLORS = ['#d9822b', '#b8452a', '#e0a93b', '#8d5a2b', '#c9652a'];

function Leaf({color, size = 10}: {color: string; size?: number}) {
  return (
    <svg width={size} height={size} viewBox="-6 -7 12 14">
      <path d="M0 -6 C4 -4 5 2 0 6 C-5 2 -4 -4 0 -6 Z" fill={color} />
      <path d="M0 -5 L0 7" stroke="#5a3a1c" strokeWidth="0.8" strokeLinecap="round" />
    </svg>
  );
}

// a few leaves lying about, the same every time
const LAWN_LEAVES = Array.from({length: 11}, (_, i) => ({
  left: (i * 37 + 7) % 100,
  bottom: (i * 5) % 7,
  rot: (i * 73) % 360,
  color: LEAF_COLORS[i % LEAF_COLORS.length],
  size: 9 + (i % 3) * 2,
}));

// the heap the hedgehog sleeps under, and where each leaf flies to when it's blown away
const PILE = Array.from({length: 18}, (_, i) => {
  const row = i < 7 ? 0 : i < 12 ? 1 : i < 16 ? 2 : 3;
  const inRow = [7, 5, 4, 2][row];
  const k = i - [0, 7, 12, 16][row];
  const x = (k - (inRow - 1) / 2) * (row === 3 ? 9 : 8) + ((i * 7) % 5) - 2;
  const side = x < 0 ? -1 : 1;
  return {
    x,
    y: row * 5,
    rot: (i * 47) % 360,
    color: LEAF_COLORS[(i * 3) % LEAF_COLORS.length],
    dx: side * (40 + ((i * 29) % 70)),
    dy: -(50 + ((i * 17) % 60)),
    spin: side * (180 + ((i * 53) % 360)),
  };
});

// a few leaves drifting down behind the page, optional. they sway and tumble, a gust blows them away. at first some
// are already on their way, after a gust new ones start at the top one after the other. the frosted cards blur them
const FALLING = Array.from({length: 7}, (_, i) => ({
  left: [7, 21, 34, 48, 62, 76, 89][i],
  size: 12 + (i % 3) * 2,
  dur: 15 + ((i * 5) % 11),
  delay: -((i * 7) % 19),
  sway: 2.6 + (i % 4) * 0.5,
  tumble: 1.8 + (i % 3) * 0.9,
  push: 140 + ((i * 37) % 120),
}));

function FallingLeaves({gusting, fresh}: {gusting: boolean; fresh: boolean}) {
  return (
    <div className={[styles.falling, gusting ? styles.gusting : ''].join(' ')} aria-hidden="true">
      {FALLING.map((l, i) => (
        <div
          key={i}
          className={styles.fall}
          style={{left: `${l.left}%`, animationDuration: `${l.dur}s`, animationDelay: `${fresh ? 1 + i * 2.2 : l.delay}s`}}
        >
          <div className={styles.gust} style={{'--push': `${l.push}px`} as React.CSSProperties}>
            <div className={styles.sway} style={{animationDuration: `${l.sway}s`}}>
              <div className={styles.tumble} style={{animationDuration: `${l.tumble}s`}}>
                <Leaf color={LEAF_COLORS[i % LEAF_COLORS.length]} size={l.size} />
              </div>
            </div>
          </div>
        </div>
      ))}
    </div>
  );
}

// the grass along the bottom. it lies behind the page, so taps on it are caught on the window:
// anything in the strip that isn't part of the ui counts. two quick taps on the heap of leaves in autumn start the
// mower, on the snowman in winter father christmas
export default function Grass() {
  const [phase, setPhase] = useState<'idle' | 'peek' | 'mowing' | 'growing' | 'santaRun' | 'snowing'>('idle');
  const [dur, setDur] = useState(0);
  // where the hedgehog sits and when the mower gets there (ms after it starts)
  // gap: the stretch the eater flies over, its grass stays
  const [critter, setCritter] = useState<{x: number; hit: number; gap: [number, number]; keep: [number, number]} | null>(null);
  // the night walk: where it ends (x, like critter), how far right it starts and where it stops to sniff (px from x),
  // and the stretch it's on now
  const [walk, setWalk] = useState<{x: number; from: number; mid: number; step: 'in' | 'sniff' | 'on' | 'hide'} | null>(null);
  // ms into father christmas's run when he gets to the snowman
  const [hello, setHello] = useState(0);
  const [gusting, setGusting] = useState(false);
  // gusts so far, the leaves in the air are new after each
  const [gusts, setGusts] = useState(0);
  const strip = useRef<HTMLDivElement>(null);
  const pile = useRef<HTMLDivElement>(null);
  const snowman = useRef<HTMLDivElement>(null);
  // the mower's position: the half of the world for the season, and the sun times. without it the north, and night
  // is 6 pm to 6 am like in the schedule
  const datum = datumFromParams(useMowerParams());
  const lat = datum?.lat;
  const lon = datum?.lon;
  // leaves on the lawn in autumn, snow in winter. the month of the viewer, not of the build
  const season = useSyncExternalStore(noChange, () => seasonOf(lat), () => null);
  const autumn = season === 'autumn';
  const winter = season === 'winter';
  const busy = useRef(false);
  const lastTap = useRef(0);
  const settings = useSyncExternalStore(settingsStore.subscribe, settingsStore.snapshot, settingsStore.serverSnapshot);
  const shown = settings.grass !== false;
  // on phones only on the dashboard, elsewhere it would sit on top of the content above the tab bar
  const dashboard = usePathname().replace(/(.)\/$/, '$1') === '/';

  useEffect(() => {
    const timers: ReturnType<typeof setTimeout>[] = [];
    const onClick = (e: MouseEvent) => {
      const r = strip.current?.getBoundingClientRect();
      if (!r || busy.current || e.clientY < r.top || e.clientY > r.bottom) return;
      const target = e.target as Element;
      if (target.closest('button, a, input, select, textarea, label, section, article, nav, svg, [role]')) return;
      if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) return;
      // the middle of the heap of leaves in autumn, of the snowman in winter
      const p = (pile.current ?? snowman.current)?.getBoundingClientRect();
      if (!p || Math.abs(e.clientX - (p.left + p.width / 2)) > 22) return;
      if (e.timeStamp - lastTap.current > 400) {
        lastTap.current = e.timeStamp;
        return;
      }
      lastTap.current = 0;
      busy.current = true;
      const w = window.innerWidth;
      const x = Math.round(p.left + p.width / 2);
      if (snowman.current) {
        // he walks from -70 to w + 10 px, the front of his mower (where the snow is cut) is 51 px in
        const ms = Math.max(4000, ((w + 80) / SANTA_SPEED) * 1000);
        setDur(ms);
        setHello(((x + 19) / (w + 80)) * ms);
        setPhase('peek');
        timers.push(setTimeout(() => setPhase('santaRun'), SANTA_PRE_MS));
        timers.push(setTimeout(() => setPhase('snowing'), SANTA_PRE_MS + ms + 300));
        timers.push(
          setTimeout(() => {
            setPhase('idle');
            busy.current = false;
          }, SANTA_PRE_MS + ms + 300 + REGROW_MS),
        );
        return;
      }
      const ms = Math.max(3000, (w / SPEED) * 1000);
      // the hedgehog sleeps under the heap
      // the mower drives from -80 to w + 10 px, its mouth (where the cut is) is 34 px in, the hedgehog
      // starts 26 px left of x
      const mouthAt = (t: number) => -46 + ((w + 90) * t) / ms;
      const hit = ((x - 26 + 46) / (w + 90)) * ms;
      // off the ground from about 0.2 to 0.8 s into the hop, which starts at HOP_LEAD before the hedgehog
      const up = hit - HOP_LEAD + 200;
      const down = hit - HOP_LEAD + 800;
      setDur(ms);
      setCritter({x, hit, gap: [mouthAt(up), mouthAt(down)], keep: [up, down - up]});
      setPhase('peek');
      timers.push(setTimeout(() => setPhase('mowing'), PRE_MS));
      timers.push(setTimeout(() => setPhase('growing'), PRE_MS + ms + 1200));
      // the grass has grown back and the hedgehog is asleep under the leaves again
      timers.push(
        setTimeout(
          () => {
            setPhase('idle');
            setCritter(null);
            busy.current = false;
          },
          Math.max(PRE_MS + ms + 1200 + REGROW_MS, PRE_MS + hit + BURY_AFTER + BURIED_MS),
        ),
      );
    };
    window.addEventListener('click', onClick);
    return () => {
      window.removeEventListener('click', onClick);
      timers.forEach(clearTimeout);
    };
  }, []);

  useEffect(() => {
    if (!shown || !autumn) return;
    const timers: ReturnType<typeof setTimeout>[] = [];
    let walking = false;
    const check = () => {
      if (busy.current || document.hidden || window.matchMedia('(prefers-reduced-motion: reduce)').matches) return;
      // not shown on phones away from the dashboard
      const p = pile.current?.getBoundingClientRect();
      if (!p || !strip.current?.getClientRects().length) return;
      const sun = lat !== undefined && lon !== undefined ? sunTimes(lat, lon) : null;
      const now = new Date();
      if (!isNightTime(clock(now), sun)) return;
      const dusk = !isNightTime(clock(new Date(now.getTime() - 90 * 60000)), sun);
      if (Math.random() > (dusk ? 0.1 : 0.05)) return;
      try {
        if (Date.now() - Number(localStorage.getItem(WALK_KEY) ?? 0) < WALK_GAP_MS) return;
        localStorage.setItem(WALK_KEY, String(Date.now()));
      } catch {}
      busy.current = walking = true;
      const w = window.innerWidth;
      const x = Math.round(p.left + p.width / 2);
      // in from just past the right edge, the hedgehog is drawn from 26 px left of x
      const from = w + 10 - (x - 26);
      const mid = Math.round(from * (0.35 + Math.random() * 0.3));
      const inMs = ((from - mid) / WALK_SPEED) * 1000;
      const onMs = (mid / WALK_SPEED) * 1000;
      setWalk({x, from, mid, step: 'in'});
      timers.push(setTimeout(() => setWalk((v) => v && {...v, step: 'sniff'}), inMs));
      timers.push(setTimeout(() => setWalk((v) => v && {...v, step: 'on'}), inMs + SNIFF_MS));
      timers.push(setTimeout(() => setWalk((v) => v && {...v, step: 'hide'}), inMs + SNIFF_MS + onMs));
      timers.push(
        setTimeout(() => {
          setWalk(null);
          busy.current = walking = false;
        }, inMs + SNIFF_MS + onMs + HIDE_MS),
      );
    };
    const timer = setInterval(check, WALK_CHECK_MS);
    return () => {
      clearInterval(timer);
      timers.forEach(clearTimeout);
      if (walking) {
        setWalk(null);
        busy.current = false;
      }
    };
  }, [shown, autumn, lat, lon]);

  useEffect(() => {
    if (!autumn) return;
    let wait: ReturnType<typeof setTimeout>;
    let calm: ReturnType<typeof setTimeout>;
    const next = () => {
      wait = setTimeout(
        () => {
          // not while the mower runs over the lawn, the grass is busy then
          if (!busy.current && !document.hidden && !window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
            setGusting(true);
            calm = setTimeout(() => {
              setGusting(false);
              setGusts((n) => n + 1);
            }, GUST_MS);
          }
          next();
        },
        GUST_MIN_MS + Math.random() * (GUST_MAX_MS - GUST_MIN_MS),
      );
    };
    next();
    return () => {
      clearTimeout(wait);
      clearTimeout(calm);
      setGusting(false);
    };
  }, [autumn]);

  const falling =
    settings.leaves === false ? null : autumn ? (
      <FallingLeaves key={gusts} gusting={gusting} fresh={gusts > 0} />
    ) : winter ? (
      <FallingSnow />
    ) : null;
  if (!shown) return falling;

  return (
    <>
    {falling}
    <div
      ref={strip}
      className={[
        styles.grass,
        phase !== 'idle' ? styles[phase] : '',
        gusting ? styles.gusting : '',
        dashboard ? '' : styles.desktopOnly,
      ].join(' ')}
      style={
        {
          '--dur': `${dur}ms`,
          '--t-hop': `${Math.round((critter?.hit ?? 0) - HOP_LEAD)}ms`,
          '--gap-from': `${Math.round(critter?.gap[0] ?? 0)}px`,
          '--gap-to': `${Math.round(critter?.gap[1] ?? 0)}px`,
          '--keep-at': `${Math.round(critter?.keep[0] ?? 0)}ms`,
          '--keep-dur': `${Math.round(critter?.keep[1] ?? 0)}ms`,
          '--t-panic': `${Math.round(Math.max(PRE_MS - 200, PRE_MS + (critter?.hit ?? 0) - 1000))}ms`,
          '--t-curl': `${Math.round(Math.max(PRE_MS + 300, PRE_MS + (critter?.hit ?? 0) - 500))}ms`,
          '--t-relief': `${Math.round(PRE_MS + (critter?.hit ?? 0) + 450)}ms`,
          '--t-sigh': `${Math.round(PRE_MS + (critter?.hit ?? 0) + SIGH_AFTER)}ms`,
          '--t-cross': `${Math.round(PRE_MS + (critter?.hit ?? 0) + CROSS_AFTER)}ms`,
          '--t-yawn': `${Math.round(PRE_MS + (critter?.hit ?? 0) + YAWN_AFTER)}ms`,
          '--t-bury': `${Math.round(PRE_MS + (critter?.hit ?? 0) + BURY_AFTER)}ms`,
          '--t-hello': `${Math.round(Math.max(0, hello - 500))}ms`,
        } as React.CSSProperties
      }
      aria-hidden="true"
    >
      <div className={styles.short} />
      <div className={styles.tall} />
      {autumn && (
        <div className={styles.lawnLeaves}>
          {LAWN_LEAVES.map((l, i) => (
            <div key={i} style={{left: `${l.left}%`, bottom: l.bottom, transform: `rotate(${l.rot}deg)`}}>
              <Leaf color={l.color} size={l.size} />
            </div>
          ))}
        </div>
      )}
      {gusting && (
        <div className={styles.whoosh}>
          <div className={styles.whooshSpin}>
            <Leaf color={LEAF_COLORS[0]} size={13} />
          </div>
        </div>
      )}
      {winter && (
        <>
          <div className={styles.snow} />
          <div ref={snowman} className={styles.snowman}>
            <Snowman />
          </div>
        </>
      )}
      {phase === 'santaRun' && (
        <div className={styles.santa}>
          <div className={styles.hoho}>Ho ho ho!</div>
          <Santa />
        </div>
      )}
      {/* the grass under the jump, it stays when the rest is cut */}
      {critter && phase === 'mowing' && <div className={[styles.tall, styles.keep].join(' ')} />}
      {critter && (
        <div className={styles.critter} style={{left: critter.x - 26}}>
          <Hedgehog />
        </div>
      )}
      {walk && (
        <div
          key={walk.step}
          className={[
            styles.walker,
            walk.step === 'sniff' ? styles.sniffing : styles.stepping,
            walk.step === 'in' || walk.step === 'on' ? styles.moving : '',
            walk.step === 'hide' ? styles.hiding : '',
          ].join(' ')}
          style={
            {
              left: walk.x - 26,
              '--from': `${walk.step === 'in' ? walk.from : walk.mid}px`,
              '--to': `${walk.step === 'in' || walk.step === 'sniff' ? walk.mid : 0}px`,
              '--walk-dur': `${Math.round(((walk.step === 'in' ? walk.from - walk.mid : walk.mid) / WALK_SPEED) * 1000)}ms`,
            } as React.CSSProperties
          }
        >
          <div className={styles.hog}>
            <Hedgehog />
          </div>
        </div>
      )}
      {autumn && (
        <div
          ref={pile}
          className={[styles.pile, phase !== 'idle' ? styles.blown : '', walk?.step === 'hide' ? styles.rustle : ''].join(' ')}
        >
          {PILE.map((l, i) => (
            <div
              key={i}
              style={
                {
                  left: 32 + l.x - 6,
                  bottom: l.y,
                  '--rot': `${l.rot}deg`,
                  '--dx': `${l.dx}px`,
                  '--dy': `${l.dy}px`,
                  '--spin': `${l.spin}deg`,
                  '--i': i,
                } as React.CSSProperties
              }
            >
              <Leaf color={l.color} size={12} />
            </div>
          ))}
          {/* someone sleeps under it */}
          {phase === 'idle' && !walk && (
            <span className={styles.zzz}>
              <span>z</span>
              <span>z</span>
              <span>Z</span>
            </span>
          )}
        </div>
      )}
      {phase === 'mowing' && (
        <div className={styles.mower}>
          <div className={styles.hop}>
            <LogoMark size={64} chomp bare />
          </div>
        </div>
      )}
    </div>
    </>
  );
}
