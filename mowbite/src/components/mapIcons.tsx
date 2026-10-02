// Map symbols to pick from in the settings. Mower icons face +x and span about -1..1, the caller
// scales and rotates them. A side view (side: true) has the ground at +y, the caller mirrors it when the
// mower heads left so it stays upright. A figure (upright: true) isn't turned at all, only faces left or right.
// Dock icons are drawn upright in the same unit box.
import {OPENMOWER_EDGE, OPENMOWER_PATHS} from './openmowerArt';

const mouth = (deg: number) => {
  const a = (deg * Math.PI) / 180;
  const x = Math.cos(a).toFixed(3);
  const y = Math.sin(a).toFixed(3);
  return `M0,0 L${x},${y} A1,1 0 1,1 ${x},${-y} Z`;
};

// strokes in the icon's own units, the map and the settings draw lines non-scaling
const SCALING = {vectorEffect: 'none'} as const;

// a leg from the hip at 0,0, black trousers and a brown shoe unless told otherwise, swinging while the mower drives,
// one step cycle per half metre
export const stepDur = (speed: number) => Math.min(2, Math.max(0.4, 0.5 / speed));
export const leg = (from: number, speed: number, trousers = '#141414', shoe = '#5d4037') => (
  <g transform={`rotate(${from / 4})`}>
    {speed > 0.08 && (
      <animateTransform
        attributeName="transform"
        type="rotate"
        values={`${from};${-from};${from}`}
        dur={`${stepDur(speed).toFixed(1)}s`}
        repeatCount="indefinite"
      />
    )}
    <path d="M-0.065,0 L0.065,0 L0.07,0.64 L-0.07,0.64 Z" fill={trousers} />
    <path d="M-0.075,0.62 L0.1,0.63 Q0.21,0.65 0.2,0.76 L-0.075,0.76 Z" fill={shoe} />
  </g>
);

// a wheel of the push mower with a cross in it, rolling along as fast as the mower goes
const pushWheel = (cx: number, cy: number, r: number, speed: number) => (
  <g>
    <circle cx={cx} cy={cy} r={r} fill="#212121" />
    <g>
      {speed > 0.05 && (
        <animateTransform
          attributeName="transform"
          type="rotate"
          values={`0 ${cx} ${cy};360 ${cx} ${cy}`}
          dur={`${Math.min(3, Math.max(0.3, 0.4 / speed)).toFixed(1)}s`}
          repeatCount="indefinite"
        />
      )}
      <path
        d={`M${cx - r * 0.8},${cy} L${cx + r * 0.8},${cy} M${cx},${cy - r * 0.8} L${cx},${cy + r * 0.8}`}
        stroke="#757575"
        strokeWidth={0.035}
        style={SCALING}
      />
    </g>
    <circle cx={cx} cy={cy} r={r * 0.42} fill="#9e9e9e" />
  </g>
);

// a cordless push mower: grass box, rounded hood with the battery on top, big rear wheels. the handle ends at about
// -0.08,-0.2, the deck runs from 0.28 to 1.04, the wheels stand on 0.88
export const pushMower = (speed: number, hood = 'var(--c-mower)') => (
  <>
    <path d="M0.1,0.2 L0.34,0.24 L0.36,0.52 L0.14,0.52 Q0.08,0.36 0.1,0.2 Z" fill="#546e7a" />
    <path d="M0.14,0.3 L0.33,0.32 M0.13,0.4 L0.34,0.42" stroke="#78909c" strokeWidth={0.03} style={SCALING} />
    <path d="M-0.12,-0.14 L-0.02,-0.18 Q0.16,0.1 0.42,0.4 L0.34,0.44 Q0.06,0.14 -0.12,-0.14 Z" fill="#37474f" />
    <path d="M-0.16,-0.2 L0.0,-0.24 L0.02,-0.16 L-0.14,-0.12 Z" fill="#212121" />
    <path
      d="M0.3,0.62 L0.3,0.44 Q0.36,0.28 0.62,0.27 Q0.92,0.27 1.0,0.46 L1.02,0.62 Z"
      fill={hood}
      stroke="#fff"
      strokeWidth={0.05}
      style={SCALING}
    />
    <path d="M0.4,0.42 Q0.62,0.34 0.94,0.46" stroke="#fff" strokeOpacity={0.5} strokeWidth={0.04} fill="none" style={SCALING} />
    <rect x={0.52} y={0.2} width={0.22} height={0.09} rx={0.03} fill="#212121" />
    <circle cx={0.69} cy={0.245} r={0.018} fill="#76ff03" />
    <rect x={0.28} y={0.58} width={0.76} height={0.08} rx={0.03} fill="#263238" />
    {pushWheel(0.38, 0.72, 0.16, speed)}
    {pushWheel(0.92, 0.75, 0.12, speed)}
  </>
);

// cut grass flying from under the deck into the grass box while the blade runs
const BOX_CLIPPINGS = [
  {c: '#7cb342', d: 0},
  {c: '#9ccc65', d: 0.2},
  {c: '#558b2f', d: 0.4},
  {c: '#aed581', d: 0.6},
].map((g, i) => (
  <circle key={`b${i}`} cx={0.62} cy={0.58} r={0.035} fill={g.c} opacity={0}>
    <animate attributeName="cx" values="0.62;0.42;0.24" dur="0.8s" begin={`${g.d}s`} repeatCount="indefinite" />
    <animate attributeName="cy" values="0.58;0.12;0.3" dur="0.8s" begin={`${g.d}s`} repeatCount="indefinite" />
    <animate attributeName="opacity" values="0;1;0" dur="0.8s" begin={`${g.d}s`} repeatCount="indefinite" />
  </circle>
));

// speed in m/s, for icons that move along with the mower, emergency while the emergency stop is active
type MowerIcon = {
  key: string;
  label: string;
  side?: boolean;
  upright?: boolean;
  // blades: the blade is running, as far as the mower tells (position/json)
  draw: (o?: {speed?: number; emergency?: boolean; blades?: boolean}) => React.ReactNode;
};

// the wheel of the openmower logo is its second part, turning around its middle while the mower drives. the
// other parts are passed through as they are
const OPENMOWER_WHEEL = '416.751 153.017';
const wheel = (part: number, speed: number, el: React.ReactElement, key: string) =>
  part === 1 && speed > 0.05 ? (
    <g key={key}>
      <animateTransform
        attributeName="transform"
        type="rotate"
        values={`0 ${OPENMOWER_WHEEL};-360 ${OPENMOWER_WHEEL}`}
        dur={`${Math.min(3, Math.max(0.4, 0.5 / speed)).toFixed(1)}s`}
        repeatCount="indefinite"
      />
      {el}
    </g>
  ) : (
    <g key={key}>{el}</g>
  );

// bits of grass flying up behind the openmower while the blade runs, each on its own beat
const CLIPPINGS = [
  {x: 300, c: '#7cb342', d: 0},
  {x: 250, c: '#9ccc65', d: 0.25},
  {x: 330, c: '#558b2f', d: 0.5},
  {x: 280, c: '#aed581', d: 0.75},
].map((g, i) => (
  <circle key={`g${i}`} cx={g.x} cy={250} r={11} fill={g.c} opacity={0}>
    <animate attributeName="cx" values={`${g.x};${g.x + 150}`} dur="1s" begin={`${g.d}s`} repeatCount="indefinite" />
    <animate attributeName="cy" values="250;150;215" dur="1s" begin={`${g.d}s`} repeatCount="indefinite" />
    <animate attributeName="opacity" values="0;1;0" dur="1s" begin={`${g.d}s`} repeatCount="indefinite" />
  </circle>
));

export const MOWER_ICONS: MowerIcon[] = [
  {
    key: 'triangle',
    label: 'Arrowhead',
    draw: () => <polygon points="1,0 -0.6,0.6 -0.6,-0.6" fill="var(--c-mower)" stroke="#fff" strokeWidth={0.5} />,
  },
  {
    key: 'arrow',
    label: 'Arrow',
    draw: () => (
      <path d="M1,0 L-0.2,-0.8 L-0.2,-0.3 L-1,-0.3 L-1,0.3 L-0.2,0.3 L-0.2,0.8 Z" fill="var(--c-mower)" stroke="#fff" strokeWidth={0.5} />
    ),
  },
  {
    key: 'robot',
    label: 'Mower',
    draw: () => (
      <>
        <rect x={-1} y={-0.95} width={0.55} height={0.3} rx={0.1} fill="#222" />
        <rect x={-1} y={0.65} width={0.55} height={0.3} rx={0.1} fill="#222" />
        <rect x={-0.95} y={-0.7} width={1.9} height={1.4} rx={0.45} fill="var(--c-mower)" stroke="#fff" strokeWidth={0.5} />
        <circle cx={0.15} cy={0} r={0.32} fill="rgba(0,0,0,0.35)" />
        <circle cx={0.62} cy={0} r={0.12} fill="#fff" />
      </>
    ),
  },
  {
    key: 'dot',
    label: 'Dot',
    draw: () => (
      <>
        <line x1={0} y1={0} x2={1.1} y2={0} stroke="var(--c-mower)" strokeWidth={2} strokeLinecap="round" />
        <circle r={0.6} fill="var(--c-mower)" stroke="#fff" strokeWidth={0.5} />
      </>
    ),
  },
  {
    key: 'chomper',
    label: 'Chomper',
    draw: () => (
      <>
        <path d={mouth(35)} fill="var(--c-mower)" stroke="#000" strokeWidth={0.5}>
          <animate attributeName="d" values={`${mouth(35)};${mouth(3)};${mouth(35)}`} dur="0.4s" repeatCount="indefinite" />
        </path>
        <circle cx={0.05} cy={-0.5} r={0.12} fill="#000" />
      </>
    ),
  },
  {
    key: 'rocket',
    label: 'Rocket',
    draw: () => (
      <>
        <path d="M-0.75,-0.18 L-1.25,0 L-0.75,0.18 Z" fill="#ffb300">
          <animate attributeName="d" values="M-0.75,-0.18 L-1.25,0 L-0.75,0.18 Z;M-0.75,-0.2 L-1.45,0 L-0.75,0.2 Z;M-0.75,-0.18 L-1.25,0 L-0.75,0.18 Z" dur="0.25s" repeatCount="indefinite" />
        </path>
        <path d="M-0.55,-0.3 L-0.95,-0.65 L-0.95,-0.2 Z M-0.55,0.3 L-0.95,0.65 L-0.95,0.2 Z" fill="#555" />
        <path d="M1,0 C0.6,-0.45 -0.2,-0.4 -0.8,-0.3 L-0.8,0.3 C-0.2,0.4 0.6,0.45 1,0 Z" fill="var(--c-mower)" stroke="#fff" strokeWidth={0.5} />
        <circle cx={0.25} cy={0} r={0.15} fill="#9be7ff" stroke="#fff" strokeWidth={0.3} />
      </>
    ),
  },
  {
    key: 'gardener',
    label: 'Gardener',
    side: true,
    upright: true,
    // the middle of the push mower sits on the mower's position, the man walks behind it
    draw: ({speed = 0, emergency = false, blades = false} = {}) => {
      const walking = speed > 0.08;
      return (
        <g transform="scale(0.8) translate(-0.65 -0.46)">
          {pushMower(emergency ? 0 : speed)}
          {blades && walking && !emergency && BOX_CLIPPINGS}
          {emergency ? (
            // emergency stop: he lies next to it, holding his knee and rocking with the pain
            <g transform="translate(-0.3 0.11) scale(0.85)">
              <g>
                <animateTransform attributeName="transform" type="rotate" values="-3 -0.4 0.7;3 -0.4 0.7;-3 -0.4 0.7" dur="1.2s" repeatCount="indefinite" />
                <path d="M-0.2,0.58 L0.14,0.6 L0.15,0.72 L-0.2,0.7 Z" fill="#141414" />
                <path d="M0.1,0.58 L0.22,0.59 Q0.27,0.66 0.24,0.73 L0.12,0.72 Z" fill="#5d4037" />
                <path d="M-0.2,0.5 L-0.1,0.44 L0.02,0.18 L0.12,0.22 L0.12,0.56 L0.02,0.56 L0.03,0.34 L-0.08,0.62 L-0.2,0.64 Z" fill="#141414" />
                <path d="M0.0,0.5 L0.13,0.5 Q0.18,0.56 0.14,0.62 L0.0,0.6 Z" fill="#5d4037" />
                <path d="M-0.72,0.44 Q-0.75,0.72 -0.68,0.72 L-0.18,0.72 L-0.16,0.46 Z" fill="#43a047" />
                <path d="M-0.5,0.46 L-0.44,0.4 L0.02,0.22 L0.04,0.3 Z" fill="#43a047" />
                <circle cx={0.04} cy={0.25} r={0.07} fill="#f1c27d" />
                <circle cx={-0.88} cy={0.56} r={0.16} fill="#f1c27d" />
                <path d="M-0.94,0.4 Q-1.1,0.5 -1.06,0.68 L-1.08,0.76 L-1.02,0.76 L-1.02,0.42 Z" fill="#c9a45c" />
              </g>
              <path
                d="M0.12,0.02 L0.2,-0.1 M0.24,0.12 L0.38,0.06 M0.0,0.0 L-0.02,-0.14"
                stroke="#ff5252"
                strokeWidth={0.05}
                strokeLinecap="round"
                style={SCALING}
              >
                <animate attributeName="opacity" values="1;0.2;1" dur="0.6s" repeatCount="indefinite" />
              </path>
            </g>
          ) : (
            // the man pushing it. walking he bobs along with his steps and now and then wipes the sweat off his
            // forehead, standing he lifts his hat now and then
            <>
              <g transform="translate(-0.45 0.12)">
                {leg(22, walking ? speed : 0)}
                {leg(-22, walking ? speed : 0)}
              </g>
              <g>
                {walking && (
                  <animateTransform
                    attributeName="transform"
                    type="translate"
                    values="0 0;0 -0.035;0 0"
                    dur={`${(stepDur(speed) / 2).toFixed(2)}s`}
                    repeatCount="indefinite"
                  />
                )}
                <path d="M-0.6,-0.5 Q-0.45,-0.6 -0.3,-0.5 L-0.28,0.16 L-0.62,0.16 Z" fill="#43a047" />
                <path d="M-0.42,-0.46 L-0.34,-0.52 L-0.04,-0.18 L-0.1,-0.1 Z" fill="#43a047" />
                <circle cx={-0.06} cy={-0.14} r={0.07} fill="#f1c27d" />
                <circle cx={-0.43} cy={-0.72} r={0.17} fill="#f1c27d" />
                <path d="M-0.64,-0.76 Q-0.43,-1.08 -0.22,-0.76 L-0.14,-0.74 L-0.14,-0.7 L-0.66,-0.7 Z" fill="#c9a45c">
                  {!walking && (
                    <animateTransform
                      attributeName="transform"
                      type="translate"
                      values="0 0;0 0;0 -0.16;0 -0.16;0 0;0 0"
                      keyTimes="0;0.8;0.84;0.92;0.96;1"
                      dur="7s"
                      repeatCount="indefinite"
                    />
                  )}
                </path>
                {walking && (
                  <>
                    <path d="M-0.27,-0.66 Q-0.22,-0.58 -0.27,-0.55 Q-0.32,-0.58 -0.27,-0.66 Z" fill="#81d4fa" opacity={0}>
                      <animate attributeName="opacity" values="0;0;1;1;0;0" keyTimes="0;0.76;0.78;0.86;0.9;1" dur="9s" repeatCount="indefinite" />
                      <animateTransform attributeName="transform" type="translate" values="0 0;0 0;0 0.25;0 0.25" keyTimes="0;0.8;0.9;1" dur="9s" repeatCount="indefinite" />
                    </path>
                    <g opacity={0}>
                      <path d="M-0.44,-0.5 L-0.36,-0.5 L-0.16,-0.4 L-0.24,-0.74 L-0.3,-0.72 L-0.24,-0.47 L-0.42,-0.4 Z" fill="#388e3c" />
                      <circle cx={-0.27} cy={-0.75} r={0.07} fill="#f1c27d" />
                      <animate attributeName="opacity" values="0;0;1;1;0;0" keyTimes="0;0.8;0.81;0.95;0.96;1" dur="9s" repeatCount="indefinite" />
                      <animateTransform
                        attributeName="transform"
                        type="rotate"
                        values="0 -0.2 -0.42;0 -0.2 -0.42;-14 -0.2 -0.42;10 -0.2 -0.42;-14 -0.2 -0.42;10 -0.2 -0.42;0 -0.2 -0.42"
                        keyTimes="0;0.82;0.85;0.88;0.91;0.94;1"
                        dur="9s"
                        repeatCount="indefinite"
                      />
                    </g>
                  </>
                )}
              </g>
            </>
          )}
        </g>
      );
    },
  },
  {
    key: 'openmower',
    label: 'OpenMower',
    side: true,
    draw: ({speed = 0, emergency = false, blades = false} = {}) => (
      // like the logo: black lines between the parts, a black edge and a white one around it so it shows on any map.
      // the edges grow with the icon (the map draws lines at a fixed width otherwise). driving, the wheel turns as fast
      // as the mower goes, with the blade on grass flies out behind it, on an emergency stop it shakes and the lines
      // flash red
      <g transform="scale(-0.0042 0.0042) translate(-269 -136)" strokeLinejoin="round">
        <g>
          {emergency && (
            <animateTransform attributeName="transform" type="translate" values="0 0;9 -3;-9 3;6 0;0 0" dur="0.35s" repeatCount="indefinite" />
          )}
          {OPENMOWER_EDGE.map((p, i) => wheel(i, speed, <path transform={p.transform} d={p.d} fill="none" stroke="#fff" strokeWidth={26} style={SCALING} />, `w${i}`))}
          {OPENMOWER_PATHS.map((p, i) =>
            wheel(
              i,
              speed,
              <path transform={p.transform} d={p.d} fill="#000" stroke="#000" strokeWidth={16} style={SCALING}>
                {emergency && (
                  <>
                    <animate attributeName="fill" values="#000;#e53935;#000" dur="0.8s" repeatCount="indefinite" />
                    <animate attributeName="stroke" values="#000;#e53935;#000" dur="0.8s" repeatCount="indefinite" />
                  </>
                )}
              </path>,
              `b${i}`,
            ),
          )}
          {OPENMOWER_PATHS.map((p, i) => wheel(i, speed, <path transform={p.transform} d={p.d} fill="var(--c-mower)" />, `c${i}`))}
          {blades && !emergency && speed > 0.05 && CLIPPINGS}
        </g>
      </g>
    ),
  },
];

export const DOCK_ICONS: {key: string; label: string; draw: () => React.ReactNode}[] = [
  {key: 'dot', label: 'Dot', draw: () => <circle r={0.65} fill="var(--c-dock)" />},
  {
    key: 'bolt',
    label: 'Charger',
    draw: () => (
      <>
        <rect x={-0.9} y={-0.9} width={1.8} height={1.8} rx={0.4} fill="var(--c-dock)" stroke="#fff" strokeWidth={0.5} />
        <path d="M0.15,-0.7 L-0.4,0.1 L-0.02,0.1 L-0.15,0.7 L0.4,-0.1 L0.02,-0.1 Z" fill="#fff" />
      </>
    ),
  },
  {
    key: 'house',
    label: 'House',
    draw: () => (
      <>
        <path d="M0,-1 L1,-0.1 L0.75,-0.1 L0.75,0.9 L-0.75,0.9 L-0.75,-0.1 L-1,-0.1 Z" fill="var(--c-dock)" stroke="#fff" strokeWidth={0.5} />
        <rect x={-0.22} y={0.3} width={0.44} height={0.6} fill="#fff" />
      </>
    ),
  },
  {
    key: 'plug',
    label: 'Plug',
    draw: () => (
      <>
        <circle r={0.9} fill="var(--c-dock)" stroke="#fff" strokeWidth={0.5} />
        <rect x={-0.4} y={-0.35} width={0.2} height={0.7} rx={0.08} fill="#fff" />
        <rect x={0.2} y={-0.35} width={0.2} height={0.7} rx={0.08} fill="#fff" />
      </>
    ),
  },
  {
    key: 'ghost',
    label: 'Ghost',
    draw: () => (
      <>
        <path
          d="M-0.85,0.9 L-0.85,-0.1 C-0.85,-1.05 0.85,-1.05 0.85,-0.1 L0.85,0.9 L0.57,0.65 L0.28,0.9 L0,0.65 L-0.28,0.9 L-0.57,0.65 Z"
          fill="var(--c-dock)"
          stroke="#fff"
          strokeWidth={0.5}
        />
        <circle cx={-0.3} cy={-0.2} r={0.22} fill="#fff" />
        <circle cx={0.3} cy={-0.2} r={0.22} fill="#fff" />
        <circle cx={-0.24} cy={-0.18} r={0.1} fill="#1a237e" />
        <circle cx={0.36} cy={-0.18} r={0.1} fill="#1a237e" />
      </>
    ),
  },
  {
    key: 'flag',
    label: 'Flag',
    draw: () => (
      <>
        <line x1={-0.6} y1={1} x2={-0.6} y2={-1} stroke="#fff" strokeWidth={1.5} strokeLinecap="round" />
        <path d="M-0.55,-0.95 L0.9,-0.6 L-0.55,-0.2 Z" fill="var(--c-dock)" stroke="#fff" strokeWidth={0.4} />
      </>
    ),
  },
  {
    key: 'shed',
    label: 'Garden shed',
    draw: () => (
      <>
        <rect x={-0.78} y={-0.2} width={1.56} height={1.12} fill="#a1714a" stroke="#fff" strokeWidth={0.06} />
        <path d="M-0.78,0.08 L0.78,0.08 M-0.78,0.36 L0.78,0.36 M-0.78,0.64 L0.78,0.64" stroke="#7b5233" strokeWidth={0.05} />
        <path d="M-1,-0.12 L0,-1 L1,-0.12 L0.82,-0.12 L0,-0.82 L-0.82,-0.12 Z" fill="var(--c-dock)" stroke="#fff" strokeWidth={0.06} />
        <path d="M-0.82,-0.12 L0,-0.82 L0.82,-0.12 Z" fill="var(--c-dock)" opacity={0.7} />
        <rect x={-0.22} y={0.22} width={0.44} height={0.7} fill="#5d3b22" />
        <circle cx={0.13} cy={0.58} r={0.04} fill="#ffd54f" />
        <rect x={0.36} y={0.02} width={0.3} height={0.26} fill="#bbdefb" stroke="#5d3b22" strokeWidth={0.05} />
        <rect x={-0.66} y={0.02} width={0.3} height={0.26} fill="#bbdefb" stroke="#5d3b22" strokeWidth={0.05} />
      </>
    ),
  },
];

export function mowerIcon(key: string | undefined) {
  return MOWER_ICONS.find((i) => i.key === key) ?? MOWER_ICONS[0];
}

export function dockIcon(key: string | undefined) {
  return DOCK_ICONS.find((i) => i.key === key) ?? DOCK_ICONS[0];
}
