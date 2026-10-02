'use client';

import type {Area} from '@/app/map/editing';
import {dayLabel, parseDay} from '@/lib/dates';
import {tr} from '@/lib/i18n';
import {localDay, type Schedule} from '@/lib/schedule';
import {useState} from 'react';
import styles from './page.module.css';

// days off: the whole schedule or some areas, up to and including a day, afterwards it runs as usual again
export default function PauseCard({schedule, mowAreas, update}: {schedule: Schedule; mowAreas: Area[]; update: (s: Schedule) => void}) {
  const today = localDay();
  const pause = schedule.pause && schedule.pause.until >= today ? schedule.pause : undefined;
  // what the next pause covers, all areas unless some are picked
  const [picked, setPicked] = useState<string[]>([]);
  const names = (ids: string[]) =>
    mowAreas
      .filter((a) => ids.includes(a.id))
      .map((a) => a.properties.name || tr('unnamed'))
      .join(', ');
  const until = (day: string) => (day === today ? tr('today') : dayLabel(parseDay(day.replaceAll('-', ''))));
  const start = (day: string) => update({...schedule, pause: {until: day, areas: picked.length ? picked : 'all'}});

  return (
    <section className={styles.card}>
      <h2>{tr('Pause')}</h2>
      {!schedule.enabled && <p className={styles.dim}>{tr("The schedule is switched off, a pause only matters once it's on.")}</p>}
      {pause ? (
        <div className={styles.row}>
          <strong>
            {pause.areas === 'all'
              ? tr('No mowing until {day}, including that day.', {day: until(pause.until)})
              : tr('{names} left out until {day}, including that day.', {names: names(pause.areas), day: until(pause.until)})}
          </strong>
          <button className={styles.add} onClick={() => update({...schedule, pause: undefined})}>
            {tr('End pause')}
          </button>
        </div>
      ) : (
        <p className={styles.dim}>{tr('Leave out mowing for a few days, e.g. a party, new seed or a holiday. Afterwards the schedule runs as usual again.')}</p>
      )}
      {mowAreas.length > 1 && (
        <div className={styles.areas}>
          <button className={picked.length === 0 ? styles.on : undefined} onClick={() => setPicked([])}>
            {tr('All areas')}
          </button>
          {mowAreas.map((a) => {
            const on = picked.includes(a.id);
            return (
              <button key={a.id} className={on ? styles.on : undefined} onClick={() => setPicked(on ? picked.filter((x) => x !== a.id) : [...picked, a.id])}>
                {a.properties.name || tr('unnamed')}
              </button>
            );
          })}
        </div>
      )}
      <div className={styles.areas}>
        <button onClick={() => start(today)}>{tr('Today')}</button>
        <label className={styles.until}>
          {tr('until')}
          <input type="date" min={today} onChange={(e) => e.target.value && start(e.target.value)} />
        </label>
      </div>
    </section>
  );
}
