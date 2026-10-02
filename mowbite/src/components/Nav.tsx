'use client';

import {startSensorHistory} from '@/hooks/useSensorHistory';
import {syncSettings} from '@/lib/settings';
import LogoMark from './Logo';
import Link from 'next/link';
import {usePathname} from 'next/navigation';
import {useEffect} from 'react';
import styles from './Nav.module.css';
import {tr, useLang} from '@/lib/i18n';
import MowerSwitch from './MowerSwitch';
import {useUnsavedMap} from '@/lib/unsavedMap';

const ICONS = {
  dashboard: (
    <>
      <path d="M6.3 18.7a8 8 0 1 1 11.4 0" />
      <path d="m12 13 3.5-3.5" />
      <circle cx="12" cy="13" r="1.2" />
    </>
  ),
  map: (
    <>
      <path d="M9 4 3 6v14l6-2 6 2 6-2V4l-6 2-6-2Z" />
      <path d="M9 4v14M15 6v14" />
    </>
  ),
  sensors: <path d="M3 12h4l3-7 4 14 3-7h4" />,
  activity: <path d="M9 6h12M9 12h12M9 18h12M4 6h.01M4 12h.01M4 18h.01" />,
  schedule: (
    <>
      <rect x="3.5" y="5" width="17" height="15" rx="2.5" />
      <path d="M3.5 10h17M8 3v4M16 3v4M12 13v3l2 1.5" />
    </>
  ),
  settings: (
    <>
      <circle cx="12" cy="12" r="3" />
      <path d="M12 3v3M12 18v3M3 12h3M18 12h3M5.6 5.6l2.1 2.1M16.3 16.3l2.1 2.1M5.6 18.4l2.1-2.1M16.3 7.7l2.1-2.1" />
    </>
  ),
};

const LINKS = [
  {href: '/', label: 'Dashboard', icon: ICONS.dashboard},
  {href: '/map', label: 'Map', icon: ICONS.map},
  {href: '/sensors', label: 'Sensors', icon: ICONS.sensors},
  {href: '/schedule', label: 'Schedule', icon: ICONS.schedule},
  {href: '/activity', label: 'Activity', icon: ICONS.activity},
];

export default function Nav() {
  // the static export uses trailing slashes, /map/ has to match /map
  const pathname = usePathname().replace(/(.)\/$/, '$1');
  useLang();
  const unsavedMap = useUnsavedMap();
  // nav is always mounted, so sensor history records no matter which page is open
  useEffect(() => {
    startSensorHistory();
    void syncSettings();
  }, []);
  return (
    <nav className={styles.nav}>
      <Link href="/" className={styles.logo} aria-label="MowBite">
        <LogoMark />
        <span className={styles.wordmark}>
          <strong>mow</strong>bite
        </span>
      </Link>
      <MowerSwitch className={styles.mowerSwitch} />
      {LINKS.map((link) => (
        <Link key={link.href} href={link.href} className={pathname === link.href ? styles.active : undefined}>
          <svg viewBox="0 0 24 24" aria-hidden="true">
            {link.icon}
          </svg>
          <span>{tr(link.label)}</span>
          {link.href === '/map' && unsavedMap && <i className={styles.unsaved} title={tr('Unsaved changes')} />}
        </Link>
      ))}
      <Link href="/settings" className={[styles.settings, pathname === '/settings' ? styles.active : ''].join(' ')}>
        <svg viewBox="0 0 24 24" aria-hidden="true">
          {ICONS.settings}
        </svg>
        <span>{tr('Settings')}</span>
      </Link>
    </nav>
  );
}
