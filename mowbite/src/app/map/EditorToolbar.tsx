import {CheckIcon, PencilIcon, RouteIcon, UndoIcon} from '@/components/icons';
import {tr} from '@/lib/i18n';
import Link from 'next/link';
import styles from './page.module.css';

// new area (only while nothing else is going on), undo and save
export default function EditorToolbar({
  idle,
  canUndo,
  unsaved,
  saving,
  saveLabel,
  saveError,
  onDraw,
  onUndo,
  onSave,
}: {
  idle: boolean;
  canUndo: boolean;
  // changed from the mower's map
  unsaved: boolean;
  saving: boolean;
  saveLabel: string;
  saveError: string | null;
  onDraw: () => void;
  onUndo: () => void;
  onSave: () => void;
}) {
  const tool = [styles.pillButton, styles.tool].join(' ');
  return (
    <div className={styles.toolbar}>
      {idle && (
        <>
          <span className={styles.toolLabel}>{tr('New area')}</span>
          <button className={tool} onClick={onDraw}>
            <PencilIcon size={16} />
            {tr('Draw')}
          </button>
          {/* recording adds the area on the mower, unsaved edits here would overwrite it */}
          {unsaved ? (
            <span className={[tool, styles.disabledLink].join(' ')} title={tr('Save or undo your changes first')}>
              <RouteIcon size={16} />
              {tr('Record')}
            </span>
          ) : (
            <Link href="/record" className={tool}>
              <RouteIcon size={16} />
              {tr('Record')}
            </Link>
          )}
          <span className={styles.toolLabel}>{tr('Changes')}</span>
        </>
      )}
      <button className={tool} onClick={onUndo} disabled={!canUndo}>
        <UndoIcon size={16} />
        {tr('Undo')}
      </button>
      <button
        className={[tool, styles.saveButton, unsaved ? styles.unsaved : ''].join(' ')}
        onClick={onSave}
        disabled={saving || !unsaved}
      >
        <CheckIcon size={16} />
        {saveLabel}
      </button>
      {saveError && <span className={[styles.error, styles.toolWide].join(' ')}>{saveError}</span>}
    </div>
  );
}
