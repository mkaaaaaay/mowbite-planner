// WMO weather codes (what Open-Meteo returns) as a small drawing and a word
type Kind = 'clear' | 'partly' | 'cloudy' | 'fog' | 'drizzle' | 'rain' | 'snow' | 'thunder';

export function weatherKind(code: number): Kind {
  if (code <= 1) return 'clear';
  if (code === 2) return 'partly';
  if (code === 3) return 'cloudy';
  if (code === 45 || code === 48) return 'fog';
  if (code >= 51 && code <= 57) return 'drizzle';
  if ((code >= 61 && code <= 67) || (code >= 80 && code <= 82)) return 'rain';
  if ((code >= 71 && code <= 77) || code === 85 || code === 86) return 'snow';
  if (code >= 95) return 'thunder';
  return 'cloudy';
}

export const WEATHER_LABELS: Record<Kind, string> = {
  clear: 'Clear',
  partly: 'Partly cloudy',
  cloudy: 'Cloudy',
  fog: 'Fog',
  drizzle: 'Drizzle',
  rain: 'Rain',
  snow: 'Snow',
  thunder: 'Thunderstorm',
};

const SUN = '#f5c542';
const CLOUD = '#c9d1d9';
const DROP = '#5b9df0';

const cloud = (x = 0, y = 0, fill = CLOUD) => (
  <path
    transform={`translate(${x} ${y})`}
    d="M7 19h10.5a4.5 4.5 0 0 0 .6-8.96A6 6 0 0 0 6.6 11.2 4 4 0 0 0 7 19Z"
    fill={fill}
  />
);

export default function WeatherIcon({code, day = true, size = 36}: {code: number; day?: boolean; size?: number}) {
  const kind = weatherKind(code);
  const sun = (r = 4.2, cx = 12, cy = 12) =>
    day ? (
      <g>
        <circle cx={cx} cy={cy} r={r} fill={SUN} />
        {Array.from({length: 8}, (_, i) => {
          const a = (i * Math.PI) / 4;
          return (
            <line
              key={i}
              x1={cx + Math.cos(a) * (r + 1.8)}
              y1={cy + Math.sin(a) * (r + 1.8)}
              x2={cx + Math.cos(a) * (r + 3.6)}
              y2={cy + Math.sin(a) * (r + 3.6)}
              stroke={SUN}
              strokeWidth="1.6"
              strokeLinecap="round"
            />
          );
        })}
      </g>
    ) : (
      <path d={`M${cx + 1} ${cy - r - 1}a${r + 1} ${r + 1} 0 1 0 ${r + 1} ${r + 2} ${r} ${r} 0 0 1 -${r + 1} -${r + 2}Z`} fill="#dfe6ee" />
    );
  const drops = (n: number, snow = false) => (
    <g stroke={snow ? '#e8eef5' : DROP} strokeWidth="1.6" strokeLinecap="round">
      {Array.from({length: n}, (_, i) =>
        snow ? (
          <circle key={i} cx={8 + i * 4} cy={21.5} r="0.9" fill="#e8eef5" />
        ) : (
          <line key={i} x1={9 + i * 3.5} y1={20.5} x2={8 + i * 3.5} y2={23} />
        ),
      )}
    </g>
  );
  return (
    <svg width={size} height={size} viewBox="0 0 24 24" aria-hidden="true">
      {kind === 'clear' && sun()}
      {kind === 'partly' && (
        <>
          {sun(3.4, 9, 9)}
          {cloud(2, 2)}
        </>
      )}
      {kind === 'cloudy' && (
        <>
          {cloud(3, -3, '#8b949e')}
          {cloud()}
        </>
      )}
      {kind === 'fog' && (
        <g stroke={CLOUD} strokeWidth="1.8" strokeLinecap="round">
          <line x1="4" y1="9" x2="20" y2="9" />
          <line x1="6" y1="13" x2="18" y2="13" />
          <line x1="4" y1="17" x2="20" y2="17" />
        </g>
      )}
      {(kind === 'drizzle' || kind === 'rain' || kind === 'snow' || kind === 'thunder') && cloud(0, -3)}
      {kind === 'drizzle' && drops(2)}
      {kind === 'rain' && drops(3)}
      {kind === 'snow' && drops(3, true)}
      {kind === 'thunder' && <path d="M12.5 15.5 10 20h2.5L11 24l4-5.5h-2.5l1.5-3Z" fill={SUN} />}
    </svg>
  );
}
