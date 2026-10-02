'use client';

import {clock} from '@/lib/dates';
import {tr} from '@/lib/i18n';
import {rosLogAround, rosLogSince, type RosLogLine} from '@/lib/roslog';
import {useEffect, useState} from 'react';
import styles from './RosLog.module.css';

// next to a problem on the activity page, when the mower keeps the log (logs.recent) or the container kept it
export function RosLogAround({t}: {t: number}) {
  const [available, setAvailable] = useState(false);
  const [open, setOpen] = useState(false);
  const [lines, setLines] = useState<RosLogLine[] | null | 'failed'>(null);

  // only for a moment the log can reach, a problem from before it was kept would just show nothing
  useEffect(() => {
    void rosLogSince().then((from) => setAvailable(from !== null && t + 60 >= from));
  }, [t]);

  if (!available) return null;

  const toggle = () => {
    setOpen(!open);
    if (!open && lines === null) void rosLogAround(t).then(setLines, () => setLines('failed'));
  };

  return (
    <>
      <button className={styles.link} onClick={toggle}>
        {open ? tr('Hide ROS log') : tr('ROS log')}
      </button>
      {open && (
        <div className={styles.log}>
          {lines === null && <span>{tr('loading…')}</span>}
          {lines === 'failed' && <span>{tr('failed')}</span>}
          {Array.isArray(lines) && lines.length === 0 && (
            <span>{tr('ROS reported nothing around that time, or it was before the log was kept (the last 14 days).')}</span>
          )}
          {Array.isArray(lines) &&
            lines.map((l, i) => (
              <span key={i} className={[styles[l.level], Math.abs(l.t - t) < 2 ? styles.at : ''].join(' ')}>
                {clock(l.t, true)} {l.level} {l.text}
              </span>
            ))}
        </div>
      )}
    </>
  );
}
