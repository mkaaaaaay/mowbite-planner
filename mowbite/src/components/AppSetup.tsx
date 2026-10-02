'use client';

import {tr, useLang} from '@/lib/i18n';
import {appMowers, saveAppMowers, selectMower} from '@/lib/mowers';
import {isApp} from '@/lib/native';
import {useState, useSyncExternalStore} from 'react';
import LogoMark from './Logo';
import styles from './AppSetup.module.css';

const never = () => () => {};

// first start of the android app: which mower to talk to
export default function AppSetup() {
  useLang();
  const app = useSyncExternalStore(never, isApp, () => false);
  const hasMower = useSyncExternalStore(never, () => appMowers().length > 0, () => true);
  const [host, setHost] = useState('');
  const [name, setName] = useState('');
  const [busy, setBusy] = useState(false);
  const [failed, setFailed] = useState(false);
  if (!app || hasMower) return null;

  const clean = () => host.trim().replace(/^https?:\/\//, '').replace(/[/:].*$/, '').toLowerCase();
  const save = () => {
    const id = Math.random().toString(36).slice(2, 10);
    saveAppMowers([{id, name: name.trim(), host: clean(), appPort: 8082, wsPort: 9001}]);
    selectMower(id);
  };
  const connect = async () => {
    setBusy(true);
    setFailed(false);
    try {
      // only whether something answers there, older MowBite versions don't allow reading from the app
      await fetch(`http://${clean()}:8082/manifest.json`, {mode: 'no-cors', signal: AbortSignal.timeout(6000)});
      save();
    } catch {
      setFailed(true);
      setBusy(false);
    }
  };

  return (
    <div className={styles.screen}>
      <div className={styles.box}>
        <LogoMark size={64} />
        <h1>
          <strong>mow</strong>bite
        </h1>
        <p>{tr('Which mower should the app connect to? Your phone has to be in the same network (or connected by VPN).')}</p>
        <label>
          {tr("The mower's address (IP or name in your network)")}
          <input value={host} onChange={(e) => setHost(e.target.value)} placeholder="192.168.2.162" autoCapitalize="off" autoCorrect="off" />
        </label>
        <label>
          {tr('Name (optional)')}
          <input value={name} onChange={(e) => setName(e.target.value)} placeholder={tr('e.g. Back garden')} />
        </label>
        {failed && (
          <p className={styles.error}>
            {tr("MowBite doesn't answer there. Check the address and that MowBite runs on the mower.")}{' '}
            <button className={styles.link} onClick={save}>
              {tr('Use it anyway')}
            </button>
          </p>
        )}
        <button className={styles.connect} onClick={() => void connect()} disabled={!clean() || busy}>
          {busy ? tr('connecting…') : tr('Connect')}
        </button>
      </div>
    </div>
  );
}
