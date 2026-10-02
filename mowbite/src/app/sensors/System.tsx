'use client';

import {TempGauge} from '@/components/gauges';
import Sparkline from '@/components/Sparkline';
import type {Sample} from '@/hooks/useSensorHistory';
import {fmt, tr} from '@/lib/i18n';
import {loadSystem, type SystemInfo} from '@/lib/system';
import {useEffect, useState} from 'react';
import styles from './page.module.css';

const gb = (bytes: number) => `${fmt(bytes / 1e9, 1)} GB`;

const uptime = (s: number) => {
  const d = Math.floor(s / 86400);
  const h = Math.floor((s % 86400) / 3600);
  return d ? `${d} d ${h} h` : `${h} h ${Math.floor((s % 3600) / 60)} min`;
};

function Bar({used, warn}: {used: number; warn?: boolean}) {
  return (
    <div className={styles.bar}>
      <div className={[styles.barFill, warn ? styles.barWarn : ''].join(' ')} style={{width: `${Math.min(100, used * 100)}%`}} />
    </div>
  );
}

// the computer side of the mower: memory, storage, cpu. from the container, updated every 30 s, the last 24 h
// from the recorder like the sensors (sys_*)
export default function System({history}: {history: Record<string, Sample[] | undefined>}) {
  // undefined: loading, null: not served
  const [sys, setSys] = useState<SystemInfo | null | undefined>(undefined);
  useEffect(() => {
    const get = () => void loadSystem().then(setSys);
    get();
    const every = setInterval(get, 30000);
    return () => clearInterval(every);
  }, []);
  if (sys === null) return <p className={styles.dim}>{tr('Only when MowBite runs as its container on the mower.')}</p>;
  if (!sys || sys.memTotal === undefined) return <p className={styles.dim}>{tr('loading…')}</p>;

  const memUsed = sys.memTotal - (sys.memAvailable ?? 0);
  // the data volume only when it's a disk of its own
  const disks = [
    {label: tr('Storage'), total: sys.diskTotal, free: sys.diskFree, history: history.sys_disk_free},
    ...(sys.dataTotal !== undefined && sys.dataTotal !== sys.diskTotal
      ? [{label: tr('Data volume'), total: sys.dataTotal, free: sys.dataFree, history: undefined}]
      : []),
  ];

  return (
    <>
      {sys.remote && <p className={styles.dim}>{tr('Of the computer MowBite runs on, not the mower.')}</p>}
      <div className={styles.grid}>
        <div className={styles.card}>
          <span className={styles.cardLabel}>{tr('Memory')}</span>
          <strong className={styles.big}>{Math.round((memUsed / sys.memTotal) * 100)} %</strong>
          <Bar used={memUsed / sys.memTotal} warn={memUsed / sys.memTotal > 0.9} />
          <span className={styles.dim}>{tr('{used} of {total} in use', {used: gb(memUsed), total: gb(sys.memTotal)})}</span>
          <Sparkline samples={history.sys_mem} digits={0} unit="%" minSpan={10} />
        </div>
        {disks.map(
          (d) =>
            d.total !== undefined &&
            d.free !== undefined && (
              <div key={d.label} className={styles.card}>
                <span className={styles.cardLabel}>{d.label}</span>
                <strong className={[styles.big, d.free < 1e9 ? styles.error : ''].join(' ')}>{tr('{free} free', {free: gb(d.free)})}</strong>
                <Bar used={1 - d.free / d.total} warn={d.free < 1e9} />
                <span className={styles.dim}>{tr('of {total}', {total: gb(d.total)})}</span>
                {d.history && <Sparkline samples={d.history} digits={1} unit="GB" minSpan={1} />}
              </div>
            ),
        )}
        {sys.cpuTemp !== undefined && (
          <div className={styles.card}>
            <span className={styles.cardLabel}>{tr('CPU temperature')}</span>
            <strong className={[styles.big, sys.cpuTemp >= 80 ? styles.error : ''].join(' ')}>{fmt(sys.cpuTemp, 0)} °C</strong>
            {/* a raspberry pi slows itself down from 80 °C on */}
            <TempGauge value={sys.cpuTemp} limits={{warn: 70, crit: 80}} />
            <Sparkline samples={history.sys_cpu_temp} digits={0} unit="°C" minSpan={5} />
          </div>
        )}
        {sys.load !== undefined && (
          <div className={styles.card}>
            <span className={styles.cardLabel}>{tr('CPU load')}</span>
            <strong className={styles.big}>{fmt(sys.load, 2)}</strong>
            {sys.cpus !== undefined && <span className={styles.dim}>{tr('{n} cores, one per core is full load', {n: sys.cpus})}</span>}
            <Sparkline samples={history.sys_load} digits={2} minSpan={1} />
          </div>
        )}
        {sys.uptime !== undefined && (
          <div className={styles.card}>
            <span className={styles.cardLabel}>{tr('Running for')}</span>
            <strong className={styles.big}>{uptime(sys.uptime)}</strong>
          </div>
        )}
      </div>
    </>
  );
}
