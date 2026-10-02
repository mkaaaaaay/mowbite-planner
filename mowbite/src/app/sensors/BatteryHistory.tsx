'use client';

import Sparkline from '@/components/Sparkline';
import {chargeSamples, drainSamples, loadBattery, trend, type Cycle} from '@/lib/battery';
import {fmt, tr} from '@/lib/i18n';
import {useEffect, useState} from 'react';
import styles from './page.module.css';

// how the battery holds up over weeks: an older one takes a shorter charge and loses more volts per hour of mowing
export default function BatteryHistory() {
  const [cycles, setCycles] = useState<Cycle[] | null | undefined>(undefined);
  useEffect(() => {
    void loadBattery().then(setCycles);
  }, []);
  if (!cycles) return null;

  const charges = chargeSamples(cycles);
  const drain = drainSamples(cycles);
  const chargeTrend = trend(charges);
  const drainTrend = trend(drain);
  const change = (t: {first: number; last: number}) => Math.round(((t.last - t.first) / t.first) * 100);

  return (
    <>
      <h2 className={styles.categoryTitle}>{tr('Battery over the weeks')}</h2>
      <div className={styles.grid}>
        <div className={styles.card}>
          <span className={styles.cardLabel}>{tr('Charging time')}</span>
          <strong className={styles.big}>{charges.length ? `${fmt(charges[charges.length - 1].v, 0)} min` : '–'}</strong>
          <Sparkline samples={charges} digits={0} unit="min" minSpan={10} />
          <span className={styles.dim}>
            {chargeTrend
              ? tr('{n} % against the first weeks', {n: `${change(chargeTrend) > 0 ? '+' : ''}${change(chargeTrend)}`})
              : charges.length === 0
                ? tr('No full charge yet (from 20 minutes on the charger)')
                : charges.length === 1
                ? tr('1 full charge so far, the trend shows after a few weeks')
                : tr('{n} full charges so far, the trend shows after a few weeks', {n: charges.length})}
          </span>
        </div>
        <div className={styles.card}>
          <span className={styles.cardLabel}>{tr('Volts per hour of mowing')}</span>
          <strong className={styles.big}>{drain.length ? `${fmt(drain[drain.length - 1].v, 2)} V/h` : '–'}</strong>
          <Sparkline samples={drain} digits={2} unit="V/h" minSpan={0.5} />
          <span className={styles.dim}>
            {drainTrend
              ? tr('{n} % against the first weeks', {n: `${change(drainTrend) > 0 ? '+' : ''}${change(drainTrend)}`})
              : drain.length === 0
                ? tr('No mow with at least 15 minutes of blade time yet')
                : drain.length === 1
                ? tr('1 mow so far, the trend shows after a few weeks')
                : tr('{n} mows so far, the trend shows after a few weeks', {n: drain.length})}
          </span>
        </div>
      </div>
      <p className={styles.footnote}>
        {tr('An ageing battery takes a shorter charge and loses more volts per hour of mowing. Kept since MowBite started recording it.')}
      </p>
    </>
  );
}
