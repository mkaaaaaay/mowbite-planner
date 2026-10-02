'use client';

import type {Area} from '@/app/map/editing';
import {locale, tr} from '@/lib/i18n';
import {isNightTime, type Plan} from '@/lib/schedule';
import type {SunTimes} from '@/lib/sun';
import {useState} from 'react';
import styles from './page.module.css';

// monday first, labels from the browser so they come out in the app's language
const weekday = (d: number) => new Date(2024, 0, d).toLocaleDateString(locale(), {weekday: 'short'});

// one start time: days, from and until, areas, the two options, and the hedgehog note when it reaches into the dark
export default function PlanRow({
  plan,
  sun,
  mowAreas,
  canReset,
  onChange,
  onRemove,
}: {
  plan: Plan;
  sun: SunTimes | null;
  mowAreas: Area[];
  canReset: boolean;
  onChange: (p: Plan) => void;
  onRemove: () => void;
}) {
  // a start or end moved into the dark waits for a yes before it's saved
  const [pending, setPending] = useState<{field: 'time' | 'end'; value: string} | null>(null);
  const set = (patch: Partial<Plan>) => onChange({...plan, ...patch});

  const setTime = (field: 'time' | 'end', value: string) => {
    if (value && isNightTime(value, sun) && !plan.dark) return setPending({field, value});
    setPending(null);
    set(field === 'time' ? {time: value} : {end: value || undefined});
  };

  const time = pending?.field === 'time' ? pending.value : plan.time;
  const end = pending?.field === 'end' ? pending.value : plan.end;
  const dark = isNightTime(time, sun) || (!!end && isNightTime(end, sun));

  return (
    <div className={styles.plan}>
      <div className={styles.row}>
        <div className={styles.days}>
          {[1, 2, 3, 4, 5, 6, 7].map((d) => (
            <button
              key={d}
              className={plan.days.includes(d) ? styles.on : undefined}
              onClick={() => set({days: plan.days.includes(d) ? plan.days.filter((x) => x !== d) : [...plan.days, d]})}
            >
              {weekday(d)}
            </button>
          ))}
        </div>
        <button className={styles.remove} onClick={onRemove} aria-label="remove">
          ×
        </button>
      </div>

      <div className={styles.row}>
        <label className={styles.until}>
          {tr('Start')}
          <input type="time" value={time} onChange={(e) => e.target.value && setTime('time', e.target.value)} />
        </label>
        <label className={styles.until}>
          {tr('until')}
          <input type="time" value={end ?? ''} onChange={(e) => setTime('end', e.target.value)} />
        </label>
        {!end && <span className={styles.dimLine}>{tr('no end time, it mows until done or the battery is low')}</span>}
      </div>

      {mowAreas.length > 1 && (
        <div className={styles.areas}>
          <button className={plan.areas.length === 0 ? styles.on : undefined} onClick={() => set({areas: []})}>
            {tr('All active areas')}
          </button>
          {mowAreas.map((a) => {
            const on = plan.areas.includes(a.id);
            return (
              <button
                key={a.id}
                className={on ? styles.on : undefined}
                onClick={() =>
                  // stays as picked, also when that's every area. none picked is all active ones
                  set({areas: on ? plan.areas.filter((x) => x !== a.id) : [...plan.areas.filter((x) => mowAreas.some((m) => m.id === x)), a.id]})
                }
              >
                {a.properties.name || tr('unnamed')}
              </button>
            );
          })}
        </div>
      )}

      <div className={styles.options}>
        <label className={styles.option}>
          <input type="checkbox" checked={!!plan.sunset} onChange={(e) => set({sunset: e.target.checked || undefined})} />
          <span>
            {tr('Home at sunset')}
            <small>
              {sun ? tr('goes home when the sun sets, today at {set}', {set: sun.set}) : tr('goes home at 6 pm (no position for the sun times)')}
            </small>
          </span>
        </label>
        {canReset && (
          <label className={styles.option}>
            <input type="checkbox" checked={!!plan.fresh} onChange={(e) => set({fresh: e.target.checked || undefined})} />
            <span>
              {tr('Start from the beginning')}
              <small>
                {plan.fresh
                  ? tr('an interrupted mow is dropped, it starts again with the first area')
                  : tr('an interrupted mow (rain, battery, sent home) carries on where it stopped')}
              </small>
            </span>
          </label>
        )}
      </div>

      {pending || (dark && !plan.dark) ? (
        <div className={styles.animals}>
          <p>
            🦔{' '}
            {sun
              ? tr('The sun sets at {set} and rises at {rise} today, so this mows in the dusk or the dark.', {set: sun.set, rise: sun.rise})
              : tr('Between 6 pm and 6 am it is dusk or dark.')}{' '}
            {tr("Hedgehogs and other animals are out then. They don't run from the mower, they curl up and can get badly hurt. Better mow during the day.")}
          </p>
          <div className={styles.animalButtons}>
            <button
              onClick={() => {
                set({...(pending ? (pending.field === 'time' ? {time: pending.value} : {end: pending.value}) : {}), dark: true});
                setPending(null);
              }}
            >
              {tr('Mow anyway')}
            </button>
            {pending && <button onClick={() => setPending(null)}>{tr('Cancel')}</button>}
          </div>
          {!pending && <p className={styles.dimLine}>{tr("Until you confirm, it doesn't start in the dark and goes home at sunset.")}</p>}
        </div>
      ) : (
        plan.dark &&
        dark && (
          <p className={styles.dimLine}>
            🦔 {tr('Also mows in the dusk and dark.')}{' '}
            <button className={styles.linkButton} onClick={() => set({dark: undefined})}>
              {tr('Only by day')}
            </button>
          </p>
        )
      )}
    </div>
  );
}
