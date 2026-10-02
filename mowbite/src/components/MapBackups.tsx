'use client';

import {parseMapFile, type BackupInfo} from '@/lib/backups';
import type {MowerMap} from '@/hooks/useMowerMap';
import {dayLabel, clock} from '@/lib/dates';
import {tr} from '@/lib/i18n';
import {useRef, useState} from 'react';
import styles from './MapBackups.module.css';

export const backupLabel = (b: BackupInfo) =>
  `${dayLabel(new Date(b.t * 1000))}, ${clock(b.t)}${b.name ? ` · ${b.auto ? tr(b.name) : b.name}` : ''}`;

// list of the container's map backups plus export/import as a file
export default function MapBackups({
  backups,
  onCreate,
  onPreview,
  onDelete,
  onDownload,
  onFile,
}: {
  // null: not served by the container, no backups on the mower then
  backups: BackupInfo[] | null;
  onCreate: (name: string) => Promise<void>;
  onPreview: (b: BackupInfo) => void;
  onDelete: (b: BackupInfo) => void;
  onDownload: (b: BackupInfo | null) => void;
  onFile: (map: MowerMap, name: string) => void;
}) {
  const [name, setName] = useState('');
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [confirmDelete, setConfirmDelete] = useState<string | null>(null);
  const file = useRef<HTMLInputElement>(null);

  const create = async () => {
    setBusy(true);
    setError(null);
    try {
      await onCreate(name.trim());
      setName('');
    } catch (e) {
      setError(e instanceof Error ? e.message : tr('failed'));
    } finally {
      setBusy(false);
    }
  };

  return (
    <details className={styles.box}>
      <summary>
        {tr('Backups')}
        {backups && backups.length > 0 && <span className={styles.count}>{backups.length}</span>}
      </summary>

      {backups ? (
        <>
          <div className={styles.row}>
            <input value={name} onChange={(e) => setName(e.target.value)} placeholder={tr('name (optional)')} maxLength={60} />
            <button className={styles.button} onClick={() => void create()} disabled={busy}>
              {tr('Create backup')}
            </button>
          </div>
          <p className={styles.dim}>{tr('Before every save the current map is backed up by itself, the last 30 of those are kept.')}</p>
          {backups.length === 0 && <p className={styles.dim}>{tr('No backups yet.')}</p>}
          <ul className={styles.list}>
            {backups.map((b) => (
              <li key={b.id}>
                <div>
                  <span>{backupLabel(b)}</span>
                  <span className={styles.dim}>
                    {tr('{n} areas', {n: b.areas})}
                    {b.auto && ` · ${tr('automatic')}`}
                  </span>
                </div>
                <div className={styles.actions}>
                  <button className={styles.link} onClick={() => onPreview(b)}>
                    {tr('Preview')}
                  </button>
                  <button className={styles.link} onClick={() => onDownload(b)}>
                    {tr('Download')}
                  </button>
                  <button
                    className={[styles.link, styles.danger].join(' ')}
                    onClick={() => (confirmDelete === b.id ? onDelete(b) : setConfirmDelete(b.id))}
                    onBlur={() => setConfirmDelete(null)}
                  >
                    {confirmDelete === b.id ? tr('Really delete?') : tr('Delete')}
                  </button>
                </div>
              </li>
            ))}
          </ul>
        </>
      ) : (
        <p className={styles.dim}>{tr('Backups on the mower need MowBite running as its container. Files still work.')}</p>
      )}

      <div className={styles.row}>
        <button className={styles.button} onClick={() => onDownload(null)}>
          {tr('Download current map')}
        </button>
        <button className={styles.button} onClick={() => file.current?.click()}>
          {tr('Load from file')}
        </button>
        <input
          ref={file}
          type="file"
          accept=".json,application/json"
          hidden
          onChange={async (e) => {
            const f = e.target.files?.[0];
            e.target.value = '';
            if (!f) return;
            try {
              onFile(parseMapFile(await f.text()), f.name);
              setError(null);
            } catch {
              setError(tr("That file doesn't look like a map."));
            }
          }}
        />
      </div>
      {error && <p className={styles.error}>{error}</p>}
    </details>
  );
}
