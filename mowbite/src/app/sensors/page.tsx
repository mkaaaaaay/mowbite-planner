'use client';

import {TitleMark} from '@/components/Logo';
import {computeGaugeScale, fallbackGaugeScale, RadialGauge, TempGauge} from '@/components/gauges';
import {BatteryIcon} from '@/components/icons';
import InfoTip from '@/components/InfoTip';
import Sparkline from '@/components/Sparkline';
import {useMowerSensors, type SensorInfo} from '@/hooks/useMowerSensors';
import {useMowerState} from '@/hooks/useMowerState';
import {useSensorHistory, type Sample} from '@/hooks/useSensorHistory';
import {useState, useSyncExternalStore} from 'react';
import {sharedSettings} from '@/lib/settings';
import {batteryColor, GPS_QUALITY_LABEL, gpsQuality, isDocked, stateColor, statusText} from '@/lib/status';
import {numParam, useMowerParams} from '@/hooks/useMowerParams';
import {PARAM} from '@/lib/openmower';
import styles from './page.module.css';
import System from './System';
import BatteryHistory from './BatteryHistory';
import {fmt, tr, useLang} from '@/lib/i18n';

// sensors that get their own card instead of the generic one
const BATTERY_IDS = new Set(['om_v_battery', 'om_v_charge', 'om_charge_current', 'om_charge_state']);
const MOTOR_IDS = new Set(['om_mow_motor_current', 'om_mow_motor_rpm']);
const GPS_ID = 'om_gps_accuracy';
// xbot_positioning reports 999 when there's no fix
const NO_FIX = 999;
// left/right esc first, rest in arrival order
const TEMP_ORDER = ['om_left_esc_temp', 'om_right_esc_temp'];

function unitOf(info: SensorInfo) {
  return info.unit === 'deg.C' ? '°C' : info.unit;
}

function digitsOf(info: SensorInfo) {
  switch (info.value_description) {
    case 'TEMPERATURE':
      return 1;
    case 'REVOLUTIONS':
      return 0;
    default:
      return 2;
  }
}

function isCritical(info: SensorInfo | undefined, raw: string | undefined, currentState: string | undefined): boolean {
  if (!info || raw === undefined || info.value_type !== 'DOUBLE') return false;
  if (info.value_description === 'REVOLUTIONS' && currentState !== 'MOWING') return false;
  const value = Number(raw);
  if (Number.isNaN(value)) return false;
  // -1 = unset
  if (info.has_critical_low && info.lower_critical_value >= 0 && value <= info.lower_critical_value) return true;
  if (info.has_critical_high && info.upper_critical_value >= 0 && value >= info.upper_critical_value) return true;
  return false;
}

function Card({title, children}: {title: React.ReactNode; children: React.ReactNode}) {
  return (
    <div className={styles.card}>
      <span className={styles.cardLabel}>{title}</span>
      {children}
    </div>
  );
}

// the mower's sensors, or the computer in it (memory, storage, cpu). the last one picked is kept per device
type Tab = 'mower' | 'system';

const savedTab = (): Tab => {
  try {
    return localStorage.getItem('sensorsTab') === 'system' ? 'system' : 'mower';
  } catch {
    return 'mower';
  }
};
// the saved tab only changes by picking one here
const onTabChange = () => () => {};

export default function SensorsPage() {
  useLang();
  // read after the page came up, the prebuilt page always starts with the mower tab
  const saved = useSyncExternalStore(onTabChange, savedTab, () => 'mower' as Tab);
  const [picked, setPicked] = useState<Tab | null>(null);
  const tab = picked ?? saved;
  const pick = (t: Tab) => {
    setPicked(t);
    try {
      localStorage.setItem('sensorsTab', t);
    } catch {}
  };
  const {infos, values} = useMowerSensors();
  const {state} = useMowerState();
  const params = useMowerParams();
  const history = useSensorHistory();
  const info = (id: string) => infos.find((i) => i.sensor_id === id);
  const num = (id: string) => (values[id] !== undefined ? Number(values[id]) : undefined);

  const currentState = state?.current_state;
  const docked = isDocked(state, values['om_v_charge']);
  const mowing = currentState === 'MOWING';
  const battery = state ? Math.round(state.battery_percentage * 100) : undefined;
  const criticalCount = infos.filter((i) => isCritical(i, values[i.sensor_id], currentState)).length;

  const batteryCritical = ['om_v_battery', 'om_v_charge', 'om_charge_current'].some((id) =>
    isCritical(info(id), values[id], currentState),
  );
  const batteryTone = batteryCritical ? 'error' : batteryColor(battery ?? 0);

  const temps = infos
    .filter((i) => i.value_description === 'TEMPERATURE')
    .sort((a, b) => {
      const ai = TEMP_ORDER.indexOf(a.sensor_id);
      const bi = TEMP_ORDER.indexOf(b.sensor_id);
      return (ai === -1 ? TEMP_ORDER.length : ai) - (bi === -1 ? TEMP_ORDER.length : bi);
    });
  const others = infos.filter(
    (i) =>
      !BATTERY_IDS.has(i.sensor_id) &&
      !MOTOR_IDS.has(i.sensor_id) &&
      i.sensor_id !== GPS_ID &&
      i.value_description !== 'TEMPERATURE',
  );

  const gps = num(GPS_ID);
  const noFix = gps !== undefined && gps >= NO_FIX;
  const gpsHistory: Sample[] | undefined = history[GPS_ID]
    ?.filter((s) => s.v < NO_FIX)
    .map((s) => ({t: s.t, v: s.v * 100, lo: s.lo === undefined ? undefined : s.lo * 100, hi: s.hi === undefined ? undefined : s.hi * 100}));

  const rpmInfo = info('om_mow_motor_rpm');
  const rpm = num('om_mow_motor_rpm');
  const motorCurrent = num('om_mow_motor_current');

  return (
    <div className={styles.page}>
      <main className={styles.main}>
        <div className={styles.header}>
          <h1>
          <TitleMark />
          {tr('Sensors')}
        </h1>
          {state && (
            <span className={[styles.chip, styles[`chip-${state.emergency ? 'error' : docked ? 'success' : stateColor(currentState)}`]].join(' ')}>
              {statusText(state, docked, values['om_charge_state'])}
            </span>
          )}
          {criticalCount > 0 && <span className={styles.error}>{tr('{n} out of range', {n: criticalCount})}</span>}
        </div>

        <div className={styles.tabs}>
          <button className={tab === 'mower' ? styles.tabOn : undefined} onClick={() => pick('mower')}>
            {tr('Mower')}
          </button>
          <button className={tab === 'system' ? styles.tabOn : undefined} onClick={() => pick('system')}>
            {tr('System')}
          </button>
        </div>

        {tab === 'system' ? (
          <System history={history} />
        ) : infos.length === 0 ? (
          <p className={styles.dim}>{tr('Waiting for sensor data…')}</p>
        ) : (
          <>
            <div className={styles.grid}>
              <Card
                title={
                  <>
                    {tr('Battery')}
                    <InfoTip>
                      {tr(
                        "The percentage is worked out from the battery voltage, between the empty and full voltage set on the mower. While charging, the charger pushes the voltage up, so it drops a bit as soon as the mower leaves the dock. Under load, especially with the blade running or uphill, it sags and comes back when the mower stops. Short jumps are normal, only the trend over a whole mow says something about the battery.",
                      )}
                    </InfoTip>
                  </>
                }
              >
                <div className={styles.batteryRow}>
                  <span className={styles[`text-${batteryTone}`]}>
                    <BatteryIcon size={26} />
                  </span>
                  <strong className={[styles.big, styles[`text-${batteryTone}`]].join(' ')}>{battery ?? '–'}%</strong>
                  {values['om_charge_state'] && <span className={styles.pill}>{tr(values['om_charge_state'])}</span>}
                </div>
                <div className={styles.details}>
                  {num('om_v_battery') !== undefined && (
                    <div>
                      <span className={styles.dim}>{tr('Battery')}</span>
                      {fmt(num('om_v_battery')!, 2)} V
                    </div>
                  )}
                  {num('om_v_charge') !== undefined && docked && (
                    <div>
                      <span className={styles.dim}>{tr('Charger')}</span>
                      {fmt(num('om_v_charge')!, 1)} V
                    </div>
                  )}
                  {num('om_charge_current') !== undefined && (
                    <div>
                      <span className={styles.dim}>{tr('Charge current')}</span>
                      {fmt(num('om_charge_current')!, 2)} A
                    </div>
                  )}
                </div>
                <Sparkline samples={history['om_v_battery']} digits={2} unit="V" minSpan={0.5} />
              </Card>

              <Card title={tr('Mow motor')}>
                {mowing && rpm !== undefined ? (
                  <>
                    <strong className={[styles.big, isCritical(rpmInfo, values['om_mow_motor_rpm'], currentState) ? styles.error : ''].join(' ')}>
                      {rpm.toFixed(0)} rpm
                    </strong>
                    {rpmInfo && <RadialGauge value={rpm} scale={computeGaugeScale(rpmInfo) ?? fallbackGaugeScale(rpm)} />}
                    {motorCurrent !== undefined && <span className={styles.dim}>{fmt(motorCurrent, 2)} A</span>}
                  </>
                ) : (
                  <strong className={styles.big}>{tr('Off')}</strong>
                )}
                <Sparkline samples={history['om_mow_motor_current']} digits={2} unit="A" minSpan={1} />
              </Card>

              <Card
                title={
                  <>
                    {tr('GPS accuracy')}
                    <InfoTip>
                      {tr(
                        "How far off the position might be, as estimated by the receiver. With RTK fix it's usually a few cm. The receiver doesn't say fix or float, so it's told by the accuracy: up to 5 cm RTK fix, up to the mower's limit (max_position_accuracy) float, still usable, worse the mower ignores the position. The mower switches GPS off while it's idle or docking.",
                      )}
                    </InfoTip>
                  </>
                }
              >
                <strong className={styles.big}>
                  {gps === undefined
                    ? '–'
                    : noFix
                      ? currentState === 'IDLE' || currentState === 'DOCKING'
                        ? tr('GPS off')
                        : tr('No fix')
                      : `${fmt(gps * 100, 1)} cm`}
                </strong>
                {!noFix && gps !== undefined && (
                  <span className={styles.dim}>{tr(GPS_QUALITY_LABEL[gpsQuality(gps, numParam(params, PARAM.maxPositionAccuracy))])}</span>
                )}
                {noFix && docked && <span className={styles.dim}>{tr('in the dock')}</span>}
                <Sparkline samples={gpsHistory} digits={1} unit="cm" minSpan={5} />
              </Card>
            </div>

            {temps.length > 0 && (
              <>
                <h2 className={styles.categoryTitle}>{tr('Temperatures')}</h2>
                <div className={styles.grid}>
                  {temps.map((t) => {
                    const v = num(t.sensor_id);
                    return (
                      <Card key={t.sensor_id} title={tr(t.sensor_name)}>
                        <strong className={[styles.big, isCritical(t, values[t.sensor_id], currentState) ? styles.error : ''].join(' ')}>
                          {v !== undefined ? `${fmt(v, 1)} °C` : '–'}
                        </strong>
                        {v !== undefined && <TempGauge value={v} info={t} />}
                        <Sparkline samples={history[t.sensor_id]} digits={1} unit="°C" minSpan={5} />
                      </Card>
                    );
                  })}
                </div>
              </>
            )}

            {others.length > 0 && (
              <>
                <h2 className={styles.categoryTitle}>{tr('Other')}</h2>
                <div className={styles.grid}>
                  {others.map((o) => {
                    const v = num(o.sensor_id);
                    const numeric = o.value_type === 'DOUBLE' && v !== undefined && !Number.isNaN(v);
                    return (
                      <Card key={o.sensor_id} title={tr(o.sensor_name)}>
                        <strong className={[styles.big, isCritical(o, values[o.sensor_id], currentState) ? styles.error : ''].join(' ')}>
                          {numeric ? `${fmt(v, digitsOf(o))} ${unitOf(o)}` : (values[o.sensor_id] ?? '–')}
                        </strong>
                        {numeric && <Sparkline samples={history[o.sensor_id]} digits={digitsOf(o)} unit={unitOf(o)} />}
                      </Card>
                    );
                  })}
                </div>
              </>
            )}

            <BatteryHistory />

            <p className={styles.footnote}>
              {sharedSettings() ? tr('History covers the last 24 hours.') : tr('History covers the last hour while the app is open.')}
            </p>
          </>
        )}
      </main>
    </div>
  );
}
