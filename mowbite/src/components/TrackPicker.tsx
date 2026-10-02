'use client';

import type {JobInfo, TrackSegment} from '@/hooks/useMowHistory';
import {useDragScroll} from '@/hooks/useDragScroll';
import {dayKey, dayLabel} from '@/lib/dates';
import {useState} from 'react';
import styles from './TrackPicker.module.css';
import {locale, tr, useLang} from '@/lib/i18n';

function length(points: [number, number][]) {
  let m = 0;
  for (let i = 1; i < points.length; i++) m += Math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]);
  return m;
}

// live trail or one recorded job, days as chips and the jobs of a day as times
export default function TrackPicker({
  jobs,
  selected,
  segments,
  onSelect,
  onClearLive,
}: {
  jobs: JobInfo[];
  selected: string | null;
  segments: TrackSegment[] | null | undefined;
  onSelect: (jobId: string | null) => void;
  // shown with the live trail when there's something to clear
  onClearLive?: () => void;
}) {
  const days: {key: string; label: string; jobs: JobInfo[]}[] = [];
  for (const j of jobs) {
    const d = new Date(j.timestamp * 1000);
    const key = dayKey(d);
    const last = days[days.length - 1];
    if (last?.key === key) last.jobs.push(j);
    else days.push({key, label: dayLabel(d), jobs: [j]});
  }

  const selectedDay = days.find((d) => d.jobs.some((j) => j.job_id === selected))?.key ?? null;
  useLang();
  const daysRef = useDragScroll<HTMLDivElement>();
  const [openDay, setOpenDay] = useState<string | null>(selectedDay);
  const day = days.find((d) => d.key === (openDay ?? selectedDay));

  const mowed = segments?.filter((s) => s.attributes.blades).reduce((m, s) => m + length(s.points), 0);
  const driven = segments?.filter((s) => !s.attributes.blades).reduce((m, s) => m + length(s.points), 0);

  return (
    <div className={styles.picker}>
      <span className={styles.title}>{tr('Track')}</span>
      <div className={styles.days} ref={daysRef}>
        <button
          className={selected === null ? styles.on : undefined}
          onClick={() => {
            setOpenDay(null);
            onSelect(null);
          }}
        >
          {tr('Live')}
        </button>
        {days.map((d) => (
          <button
            key={d.key}
            className={d.key === (openDay ?? selectedDay) ? styles.on : d.key === selectedDay ? styles.half : undefined}
            onClick={() => {
              setOpenDay(d.key);
              if (d.jobs.length === 1) onSelect(d.jobs[0].job_id);
            }}
          >
            {d.label}
          </button>
        ))}
      </div>

      {day && day.jobs.length > 1 && (
        <div className={styles.times}>
          {day.jobs.map((j) => (
            <button key={j.job_id} className={j.job_id === selected ? styles.on : undefined} onClick={() => onSelect(j.job_id)}>
              {new Date(j.timestamp * 1000).toLocaleTimeString(locale(), {hour: '2-digit', minute: '2-digit'})}
            </button>
          ))}
        </div>
      )}

      {selected === null && onClearLive && (
        <button className={styles.clear} onClick={onClearLive} title={tr('Only hides it here, the mower keeps the recording.')}>
          {tr('Clear live track')}
        </button>
      )}

      {selected && (
        <span className={styles.info}>
          {segments === null || segments === undefined
            ? tr('loading…')
            : tr('{mowed} m mowed, {driven} m driven without blades', {mowed: Math.round(mowed ?? 0), driven: Math.round(driven ?? 0)})}
        </span>
      )}
    </div>
  );
}
