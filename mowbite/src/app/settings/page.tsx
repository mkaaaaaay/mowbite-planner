'use client';

import {TitleMark} from '@/components/Logo';
import {DOCK_ICONS, dockIcon, MOWER_ICONS, mowerIcon} from '@/components/mapIcons';
import {isTileUrl} from '@/lib/imagery';
import {COLORS, saveSettings, settingsStore, sharedSettings, type ColorKey, type OtherMower, type Settings} from '@/lib/settings';
import {useRouter} from 'next/navigation';
import {useState, useSyncExternalStore} from 'react';
import styles from './page.module.css';
import {setLangChoice, tr, useLang, useLangChoice} from '@/lib/i18n';
import {setThemeChoice, useThemeChoice} from '@/lib/theme';
import {appMowers, saveAppMowers} from '@/lib/mowers';
import {isApp} from '@/lib/native';
import {setSwipeEnabled, useSwipeEnabled} from '@/lib/swipe';
import {UpdateSettings} from '@/components/Updates';

function IconChoice({
  icons,
  value,
  onChange,
  rotate,
}: {
  icons: {key: string; label: string; upright?: boolean; draw: () => React.ReactNode}[];
  value: string;
  onChange: (key: string) => void;
  rotate?: boolean;
}) {
  return (
    <div className={styles.icons}>
      {icons.map((icon) => (
        <button
          key={icon.key}
          className={[styles.icon, icon.key === value ? styles.iconOn : ''].join(' ')}
          onClick={() => onChange(icon.key)}
          title={tr(icon.label)}
        >
          <svg viewBox="-1.5 -1.5 3 3" className={styles.preview}>
            <g transform={rotate && !icon.upright ? 'rotate(-30)' : undefined}>{icon.draw()}</g>
          </svg>
          <span>{tr(icon.label)}</span>
        </button>
      ))}
    </div>
  );
}

function SizeSlider({value, onChange}: {value: number; onChange: (v: number) => void}) {
  return (
    <label className={styles.size}>
      {tr('Size')}
      <input type="range" min={0.5} max={3} step={0.1} value={value} onChange={(e) => onChange(+e.target.value)} />
      <span>{Math.round(value * 100)}%</span>
      {value !== 1 && (
        <button className={styles.linkButton} onClick={() => onChange(1)}>
          {tr('default')}
        </button>
      )}
    </label>
  );
}

// both icons on a bit of lawn at about the size they have on a map filling a phone screen
function IconPreview({icons}: {icons: NonNullable<Settings['icons']>}) {
  return (
    <svg viewBox="0 0 220 76" className={styles.lawn}>
      <rect x="0" y="0" width="220" height="76" fill="var(--c-mow)" opacity="0.18" />
      <path d="M48 40 C 90 70, 120 10, 160 36" className={styles.lawnTrack} />
      <g transform={`translate(48 40) scale(${10 * (icons.dockSize ?? 1)})`}>{dockIcon(icons.dock).draw()}</g>
      <g transform={`translate(160 36) rotate(${mowerIcon(icons.mower).upright ? 0 : -20}) scale(${10 * (icons.mowerSize ?? 1)})`}>
        {mowerIcon(icons.mower).draw()}
      </g>
    </svg>
  );
}

const noop = () => () => {};

function MowersSection({settings}: {settings: Settings}) {
  const [name, setName] = useState('');
  const [host, setHost] = useState('');
  const [appPort, setAppPort] = useState('8082');
  const [wsPort, setWsPort] = useState('9001');
  const [error, setError] = useState<string | null>(null);
  // the android app keeps its list on the phone, a mower keeps it in its shared settings
  const app = useSyncExternalStore(noop, isApp, () => false);
  const [phoneList, setPhoneList] = useState<OtherMower[]>(() => appMowers());
  const others = app ? phoneList : (settings.mowers ?? []);
  const store = (list: OtherMower[]) => {
    if (app) {
      saveAppMowers(list);
      setPhoneList(list);
    } else saveSettings({...settingsStore.snapshot(), mowers: list});
  };
  const add = () => {
    const h = host.trim().replace(/^https?:\/\//, '').replace(/[/:].*$/, '').toLowerCase();
    if (!h) return;
    if (!app && (h === window.location.hostname.toLowerCase() || h === 'localhost' || h === '127.0.0.1')) {
      setError(tr("That's this mower, it's already there. Enter the address of another one."));
      return;
    }
    if (others.some((m) => m.host.toLowerCase() === h)) {
      setError(tr('This mower is already in the list.'));
      return;
    }
    const mower: OtherMower = {
      id: Math.random().toString(36).slice(2, 10),
      name: name.trim(),
      host: h,
      appPort: Number(appPort) || 8082,
      wsPort: Number(wsPort) || 9001,
    };
    store([...others, mower]);
    setName('');
    setHost('');
    setError(null);
  };
  return (
    <section className={styles.card}>
      <h2>{tr('Mowers')}</h2>
      <p className={styles.dim}>
        {tr(
          "Got more than one robot mower with MowBite? Add the others here, as many as you like, and switch between them at the top of the dashboard (on a computer in the sidebar). Colors, icons and the language stay the same for all.",
        )}
      </p>

      {!app && (
        <label className={styles.field}>
          {tr('This mower (the one this app runs on)')}
          <input
            key={'this' + (settings.thisName ?? '')}
            defaultValue={settings.thisName ?? ''}
            placeholder={tr('Name, e.g. Back garden')}
            onBlur={(e) => saveSettings({...settingsStore.snapshot(), thisName: e.target.value.trim() || undefined})}
          />
        </label>
      )}

      {others.length > 0 && <span className={styles.subTitle}>{tr('Other mowers')}</span>}
      {others.map((m) => (
        <div key={m.id} className={styles.mowerRow}>
          <span>
            <strong>{m.name || m.host}</strong>{' '}
            <span className={styles.dim}>
              {m.host}:{m.appPort ?? 8082}
            </span>
          </span>
          <button
            className={styles.linkButton}
            onClick={() => store(others.filter((x) => x.id !== m.id))}
          >
            {tr('Remove')}
          </button>
        </div>
      ))}

      <div className={styles.mowerAdd}>
        <span className={styles.subTitle}>{tr('Add another mower')}</span>
        <label className={styles.field}>
          {tr('Name')}
          <input value={name} onChange={(e) => setName(e.target.value)} placeholder={tr('e.g. Front garden')} />
        </label>
        <label className={styles.field}>
          {tr("Address of the other mower (IP or name in your network)")}
          <input
            value={host}
            onChange={(e) => {
              setHost(e.target.value);
              setError(null);
            }}
            placeholder="192.168.2.170"
            autoCapitalize="off"
            autoCorrect="off"
          />
        </label>
        <details className={styles.ports}>
          <summary>{tr('Ports (only if you changed them)')}</summary>
          <label className={styles.field}>
            {tr('MowBite port')}
            <input value={appPort} onChange={(e) => setAppPort(e.target.value)} inputMode="numeric" />
          </label>
          <label className={styles.field}>
            {tr('MQTT websocket port')}
            <input value={wsPort} onChange={(e) => setWsPort(e.target.value)} inputMode="numeric" />
          </label>
        </details>
        {error && <span className={styles.error}>{error}</span>}
        <button className={styles.pillButton} onClick={add} disabled={!host.trim()}>
          {tr('Add mower')}
        </button>
      </div>
    </section>
  );
}

export default function SettingsPage() {
  useLang();
  const choice = useLangChoice();
  const theme = useThemeChoice();
  const swipe = useSwipeEnabled();
  const router = useRouter();
  // back to wherever the gear was clicked, or the dashboard when the page was opened directly
  const back = () => (window.history.length > 1 ? router.back() : router.push('/'));
  const settings = useSyncExternalStore(settingsStore.subscribe, settingsStore.snapshot, settingsStore.serverSnapshot);
  const colors = settings.colors ?? {};
  const icons = settings.icons ?? {};

  // always start from the latest saved state, two quick changes shouldn't overwrite each other
  const setColor = (key: ColorKey, value: string | undefined) => {
    const cur = settingsStore.snapshot();
    const next = {...cur.colors};
    if (value === undefined) delete next[key];
    else next[key] = value;
    saveSettings({...cur, colors: next});
  };

  const setImagery = (field: 'url' | 'attribution', value: string) => {
    const cur = settingsStore.snapshot();
    saveSettings({...cur, imagery: {...cur.imagery, [field]: value.trim() || undefined}});
  };
  const imageryUrl = settings.imagery?.url;

  const setIcon = (field: 'mower' | 'dock' | 'mowerSize' | 'dockSize' | 'mowerRealSize', value: string | number | boolean) => {
    const cur = settingsStore.snapshot();
    saveSettings({...cur, icons: {...cur.icons, [field]: value}});
  };

  return (
    <div className={styles.page}>
      <main className={styles.main}>
        <button className={styles.back} onClick={back}>
          ← {tr('Back')}
        </button>
        <h1>
          <TitleMark />
          {tr('Settings')}
        </h1>
        <p className={styles.dim}>
          {sharedSettings() ? tr('Saved on the mower, the same on all your devices.') : tr('Saved on this device only.')}
        </p>

        <section className={styles.card}>
          <h2>{tr('Language')}</h2>
          <div className={styles.segment}>
            {(
              [
                ['auto', tr('Automatic')],
                ['en', 'English'],
                ['de', 'Deutsch'],
              ] as const
            ).map(([k, label]) => (
              <button key={k} className={choice === k ? styles.segmentOn : undefined} onClick={() => setLangChoice(k)}>
                {label}
              </button>
            ))}
          </div>
          <p className={styles.dim}>{tr("Only for this device. Automatic follows the browser's language.")}</p>
        </section>

        <section className={styles.card}>
          <h2>{tr('Design')}</h2>
          <div className={styles.segment}>
            {(
              [
                ['auto', tr('Automatic')],
                ['light', tr('Light')],
                ['dark', tr('Dark')],
                ['frost', tr('Frosted glass')],
              ] as const
            ).map(([k, label]) => (
              <button key={k} className={theme === k ? styles.segmentOn : undefined} onClick={() => setThemeChoice(k)}>
                {label}
              </button>
            ))}
          </div>
          <p className={styles.dim}>{tr("Only for this device. Automatic follows the phone's or computer's setting.")}</p>
        </section>

        <MowersSection settings={settings} />

        <section className={styles.card}>
          <h2>{tr('Display')}</h2>
          <label className={styles.check}>
            <input
              type="checkbox"
              checked={settings.dashboard?.map === 'always'}
              onChange={(e) => {
                const cur = settingsStore.snapshot();
                saveSettings({...cur, dashboard: {...cur.dashboard, map: e.target.checked ? 'always' : 'auto'}});
              }}
            />
            {tr('Always show the map, not only while the mower is driving')}
          </label>
          <label className={styles.check}>
            <input
              type="checkbox"
              checked={settings.grass !== false}
              onChange={(e) => saveSettings({...settingsStore.snapshot(), grass: e.target.checked})}
            />
            {tr('Grass along the bottom of the screen')}
          </label>
          <label className={styles.check}>
            <input
              type="checkbox"
              checked={settings.leaves !== false}
              onChange={(e) => saveSettings({...settingsStore.snapshot(), leaves: e.target.checked})}
            />
            {tr('Falling leaves in autumn, snow in winter')}
          </label>
          <label className={styles.check}>
            <input type="checkbox" checked={swipe} onChange={(e) => setSwipeEnabled(e.target.checked)} />
            {tr('Swipe left and right between the pages (on this device)')}
          </label>
        </section>

        <section className={styles.card}>
          <h2>{tr('Weather')}</h2>
          <label className={styles.check}>
            <input
              type="checkbox"
              checked={!!settings.weather}
              onChange={(e) => saveSettings({...settingsStore.snapshot(), weather: e.target.checked})}
            />
            {tr('Show the weather for the garden')}
          </label>
          <p className={styles.dim}>
            {tr(
              "Shows the current weather and whether rain is coming on the dashboard. The data comes from Open-Meteo, for that the garden's position is sent there, rounded to about a kilometer.",
            )}
          </p>
        </section>

        <section className={styles.card}>
          <h2>{tr('Mower icon')}</h2>
          <IconChoice icons={MOWER_ICONS} value={icons.mower ?? MOWER_ICONS[0].key} onChange={(k) => setIcon('mower', k)} rotate />
          <div className={styles.segment}>
            <span>{tr('On the map')}</span>
            <button className={!icons.mowerRealSize ? styles.segmentOn : undefined} onClick={() => setIcon('mowerRealSize', false)}>
              {tr('Same size at every zoom')}
            </button>
            <button className={icons.mowerRealSize ? styles.segmentOn : undefined} onClick={() => setIcon('mowerRealSize', true)}>
              {tr('Real size')}
            </button>
          </div>
          <SizeSlider value={icons.mowerSize ?? 1} onChange={(v) => setIcon('mowerSize', v)} />
          <IconPreview icons={icons} />
        </section>

        <section className={styles.card}>
          <h2>{tr('Docking station icon')}</h2>
          <IconChoice icons={DOCK_ICONS} value={icons.dock ?? DOCK_ICONS[0].key} onChange={(k) => setIcon('dock', k)} />
          <SizeSlider value={icons.dockSize ?? 1} onChange={(v) => setIcon('dockSize', v)} />
          <IconPreview icons={icons} />
        </section>

        <section className={styles.card}>
          <h2>{tr('Aerial imagery')}</h2>
          <p className={styles.dim}>
            {tr(
              'Official open orthophotos are built in where a country offers them for free: Germany (all states except Saarland), Austria, Switzerland, the Netherlands, Belgium, Luxembourg, France, Spain, Czechia, Finland and the USA. Anywhere else you can add a tile source you are allowed to use: an XYZ url with {z}, {x} and {y}, or a WMS url in EPSG:3857 with {bbox}. It shows up as an extra choice on the map, and you are responsible for its terms of use.',
              {z: '{z}', x: '{x}', y: '{y}', bbox: '{bbox}'},
            )}
          </p>
          {/* saved when leaving the field, not on every key */}
          <label className={styles.field}>
            {tr('Tile url')}
            <input
              key={'url' + (imageryUrl ?? '')}
              defaultValue={imageryUrl ?? ''}
              placeholder="https://example.org/tiles/{z}/{x}/{y}.jpg"
              onBlur={(e) => setImagery('url', e.target.value)}
            />
          </label>
          {imageryUrl && !isTileUrl(imageryUrl) && (
            <span className={styles.error}>{tr('The url needs to start with http(s) and contain {z}, {x} and {y}, or {bbox} for a WMS.', {z: '{z}', x: '{x}', y: '{y}', bbox: '{bbox}'})}</span>
          )}
          <label className={styles.field}>
            {tr('Attribution')}
            <input
              key={'attr' + (settings.imagery?.attribution ?? '')}
              defaultValue={settings.imagery?.attribution ?? ''}
              placeholder={tr('shown on the map, as the source requires')}
              onBlur={(e) => setImagery('attribution', e.target.value)}
            />
          </label>
        </section>

        <section className={styles.card}>
          <div className={styles.cardHead}>
            <h2>{tr('Map colors')}</h2>
            <button
              className={styles.pillButton}
              onClick={() => saveSettings({...settingsStore.snapshot(), colors: {}})}
              disabled={!Object.keys(colors).length}
            >
              {tr('Reset all')}
            </button>
          </div>
          <p className={styles.dim}>{tr('Used on every map in the app.')}</p>

          <div className={styles.colors}>
            {COLORS.map((c) => {
              const value = colors[c.key] ?? c.value;
              return (
                <div key={c.key} className={styles.colorRow}>
                  <input type="color" value={value} onChange={(e) => setColor(c.key, e.target.value)} aria-label={tr(c.label)} />
                  <span>{tr(c.label)}</span>
                  {colors[c.key] && (
                    <button className={styles.linkButton} onClick={() => setColor(c.key, undefined)}>
                      {tr('default')}
                    </button>
                  )}
                </div>
              );
            })}
          </div>
        </section>

        <UpdateSettings cardClass={styles.card} checkClass={styles.check} />
      </main>
    </div>
  );
}
