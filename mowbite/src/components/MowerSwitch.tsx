'use client';

import {tr, useLang} from '@/lib/i18n';
import {mowerList, selectedMower, selectMower} from '@/lib/mowers';
import {isApp} from '@/lib/native';
import {settingsStore} from '@/lib/settings';
import {useSyncExternalStore} from 'react';
import styles from './MowerSwitch.module.css';

const never = () => () => {};

// picks the mower this device looks at, only there once other mowers are set up in the settings
export default function MowerSwitch({className}: {className?: string}) {
  useLang();
  const settings = useSyncExternalStore(settingsStore.subscribe, settingsStore.snapshot, settingsStore.serverSnapshot);
  const app = useSyncExternalStore(never, isApp, () => false);
  const current = useSyncExternalStore(never, () => selectedMower()?.id ?? '', () => '');
  // in the app the list is on the phone and there's no mower of its own
  const others = app ? mowerList() : (settings.mowers ?? []);
  if (app ? others.length < 2 : !others.length) return null;
  return (
    <select
      className={[styles.select, className].filter(Boolean).join(' ')}
      value={others.some((m) => m.id === current) ? current : ''}
      onChange={(e) => selectMower(e.target.value)}
      aria-label={tr('Mower')}
    >
      {!app && <option value="">{settings.thisName || tr('This mower')}</option>}
      {others.map((m) => (
        <option key={m.id} value={m.id}>
          {m.name || m.host}
        </option>
      ))}
    </select>
  );
}
