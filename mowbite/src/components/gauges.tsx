'use client';

import type {SensorInfo} from '@/hooks/useMowerSensors';

export interface GaugeZone {
  from: number;
  to: number;
  color: string;
}

export interface GaugeScale {
  domainMin: number;
  domainMax: number;
  zones: GaugeZone[];
}

const RED = '#e53935';
const YELLOW = '#fbc02d';
const GREEN = '#43a047';

const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));

// -1 means unset in the firmware but has_critical_* can still be true, so ignore negative thresholds
export function computeGaugeScale(info: SensorInfo): GaugeScale | null {
  const hasCriticalLow = !!info.has_critical_low && info.lower_critical_value >= 0;
  const hasCriticalHigh = !!info.has_critical_high && info.upper_critical_value >= 0;
  const hasMinMax = !!info.has_min_max && info.min_value >= 0 && info.max_value >= 0;
  if (!hasMinMax && !hasCriticalLow && !hasCriticalHigh) return null;

  const lowBound = hasCriticalLow ? info.lower_critical_value : hasMinMax ? info.min_value : 0;
  const highBound = hasCriticalHigh ? info.upper_critical_value : hasMinMax ? info.max_value : lowBound + 1;

  const span = highBound - lowBound || 1;
  const pad = span * 0.15;
  const domainMin = Math.max(0, lowBound - pad);
  const domainMax = highBound + pad;

  const zones: GaugeZone[] = [];
  if (hasCriticalLow) {
    zones.push({from: domainMin, to: info.lower_critical_value, color: RED});
  }
  if (hasMinMax) {
    const yellowLowStart = hasCriticalLow ? info.lower_critical_value : domainMin;
    if (yellowLowStart < info.min_value) zones.push({from: yellowLowStart, to: info.min_value, color: YELLOW});
    zones.push({from: info.min_value, to: info.max_value, color: GREEN});
    const yellowHighEnd = hasCriticalHigh ? info.upper_critical_value : domainMax;
    if (info.max_value < yellowHighEnd) zones.push({from: info.max_value, to: yellowHighEnd, color: YELLOW});
  } else {
    const midStart = hasCriticalLow ? info.lower_critical_value : domainMin;
    const midEnd = hasCriticalHigh ? info.upper_critical_value : domainMax;
    if (midEnd > midStart) zones.push({from: midStart, to: midEnd, color: GREEN});
  }
  if (hasCriticalHigh) {
    zones.push({from: info.upper_critical_value, to: domainMax, color: RED});
  }

  return {domainMin, domainMax, zones};
}

export function fallbackGaugeScale(value: number): GaugeScale {
  const domainMax = value > 0 ? value * 1.5 : 1;
  return {domainMin: 0, domainMax, zones: [{from: 0, to: domainMax, color: GREEN}]};
}

const fractionOf = (value: number, scale: GaugeScale) => {
  const span = scale.domainMax - scale.domainMin || 1;
  return clamp((value - scale.domainMin) / span, 0, 1);
};

export function RadialGauge({value, scale, size = 120}: {value: number; scale: GaugeScale; size?: number}) {
  const cx = size / 2;
  const cy = size / 2;
  const r = size / 2 - 16;
  const startAngle = 135; // deg, 0 = east, clockwise
  const sweep = 270;

  const angleFor = (frac: number) => (startAngle + frac * sweep) * (Math.PI / 180);
  const point = (frac: number): [number, number] => {
    const a = angleFor(frac);
    return [cx + r * Math.cos(a), cy + r * Math.sin(a)];
  };

  const arcPath = (fromFrac: number, toFrac: number) => {
    const [x1, y1] = point(fromFrac);
    const [x2, y2] = point(toFrac);
    const large = (toFrac - fromFrac) * sweep > 180 ? 1 : 0;
    return `M ${x1} ${y1} A ${r} ${r} 0 ${large} 1 ${x2} ${y2}`;
  };

  const frac = fractionOf(value, scale);
  const needleAngle = angleFor(frac);
  const needleLen = r - 4;
  const needleX = cx + needleLen * Math.cos(needleAngle);
  const needleY = cy + needleLen * Math.sin(needleAngle);

  return (
    <svg width={size} height={size} viewBox={`0 0 ${size} ${size}`}>
      {scale.zones.map((zone, i) => (
        <path
          key={i}
          d={arcPath(fractionOf(zone.from, scale), fractionOf(zone.to, scale))}
          stroke={zone.color}
          strokeWidth={10}
          fill="none"
        />
      ))}
      <line x1={cx} y1={cy} x2={needleX} y2={needleY} stroke="#616161" strokeWidth={2} />
      <circle cx={cx} cy={cy} r={4} fill="#616161" />
    </svg>
  );
}

// Temperatures all share one fixed scale so they can be compared at a glance. Only the upper limits
// matter (a cold motor is fine): yellow above the sensor's normal max, red from its critical value.
// Sensors without limits just get a neutral bar.
// the limits from the sensor's info, or given directly (warn: yellow mark, crit: red one)
export function TempGauge({value, info, limits}: {value: number; info?: SensorInfo; limits?: {warn?: number; crit?: number}}) {
  const warn = info ? (info.has_min_max && info.max_value > 0 ? info.max_value : undefined) : limits?.warn;
  const crit = info ? (info.has_critical_high && info.upper_critical_value >= 0 ? info.upper_critical_value : undefined) : limits?.crit;
  const max = Math.max(90, (crit ?? 0) + 10);
  const color = crit !== undefined && value >= crit ? RED : warn !== undefined && value >= warn ? YELLOW : GREEN;
  const x = (v: number) => (clamp(v, 0, max) / max) * 100;
  const ticks = [0, 30, 60, 90].filter((t) => t <= max);

  return (
    <div style={{width: '100%', marginTop: 6}}>
      <svg viewBox="0 0 100 10" preserveAspectRatio="none" width="100%" height={10} style={{display: 'block'}}>
        <rect x={0} y={1.5} width={100} height={7} rx={3.5} fill="rgba(255,255,255,0.08)" />
        <rect x={0} y={1.5} width={Math.max(2, x(value))} height={7} rx={3.5} fill={color} />
        {warn !== undefined && <rect x={x(warn) - 0.4} y={0} width={0.8} height={10} fill={YELLOW} />}
        {crit !== undefined && <rect x={x(crit) - 0.4} y={0} width={0.8} height={10} fill={RED} />}
      </svg>
      <div style={{position: 'relative', height: 14, fontSize: 10, color: '#888'}}>
        {ticks.map((t) => (
          <span
            key={t}
            style={{
              position: 'absolute',
              left: `${x(t)}%`,
              transform: t === 0 ? 'none' : t === max ? 'translateX(-100%)' : 'translateX(-50%)',
            }}
          >
            {t}°
          </span>
        ))}
      </div>
    </div>
  );
}
