import type {Sample} from '@/hooks/useSensorHistory';
import styles from './Sparkline.module.css';
import {fmt, tr} from '@/lib/i18n';

// small line of the history, lowest and highest value next to it
// minSpan keeps sensor noise from looking like big swings on an otherwise flat line
export default function Sparkline({
  samples,
  digits = 1,
  unit = '',
  minSpan = 1,
}: {
  samples?: Sample[];
  digits?: number;
  unit?: string;
  minSpan?: number;
}) {
  if (!samples || samples.length < 2) return <div className={styles.empty}>{tr('collecting history…')}</div>;

  const w = 200;
  const h = 32;
  const t0 = samples[0].t;
  const t1 = samples[samples.length - 1].t;
  const lo = Math.min(...samples.map((s) => s.lo ?? s.v));
  const hi = Math.max(...samples.map((s) => s.hi ?? s.v));
  const span = Math.max(hi - lo, minSpan);
  const base = (lo + hi) / 2 - span / 2;
  const points = samples
    .map((s) => `${(((s.t - t0) / (t1 - t0 || 1)) * w).toFixed(1)},${(h - 2 - ((s.v - base) / span) * (h - 4)).toFixed(1)}`)
    .join(' ');
  const minutes = Math.max(1, Math.round((t1 - t0) / 60000));
  const range = minutes < 90 ? `${minutes} min` : minutes < 2880 ? `${Math.round(minutes / 60)} h` : `${Math.round(minutes / 1440)} d`;

  return (
    <div className={styles.wrap}>
      <svg className={styles.svg} viewBox={`0 0 ${w} ${h}`} preserveAspectRatio="none">
        <polyline points={points} />
      </svg>
      <div className={styles.legend}>
        <span>{range}</span>
        <span>
          {fmt(lo, digits)}–{fmt(hi, digits)} {unit}
        </span>
      </div>
    </div>
  );
}
