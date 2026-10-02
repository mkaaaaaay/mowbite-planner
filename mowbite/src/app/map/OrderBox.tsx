import InfoTip from '@/components/InfoTip';
import {tr} from '@/lib/i18n';
import type {Area} from './editing';
import styles from './page.module.css';

// the order the mower goes through the mowing areas in
export default function OrderBox({
  areas,
  docked,
  onSelect,
  onMove,
}: {
  areas: Area[];
  docked: boolean;
  onSelect: (id: string) => void;
  onMove: (id: string, by: number) => void;
}) {
  return (
    <div className={styles.orderBox}>
      <span className={styles.orderTitle}>
        {tr('Mowing order')}
        <InfoTip>{tr("The mower mows the areas in this order. Can only be changed while it's idle in the dock.")}</InfoTip>
      </span>
      {!docked && <span className={styles.dim}>{tr('Can only be changed while the mower is idle in the dock.')}</span>}
      {areas.map((a, i) => (
        <div key={a.id} className={styles.orderRow}>
          <span className={styles.orderNum}>{i + 1}</span>
          <a onClick={() => onSelect(a.id)}>
            {a.properties.name || tr('unnamed')}
            {a.properties.active === false && <span className={styles.dim}> ({tr('inactive')})</span>}
            {a.properties.active !== false && a.properties.mowable === false && (
              <span className={styles.dim}> ({tr("don't mow")})</span>
            )}
          </a>
          <button
            className={styles.pillButton}
            onClick={() => onMove(a.id, -1)}
            disabled={i === 0 || !docked}
            aria-label="earlier"
          >
            ↑
          </button>
          <button
            className={styles.pillButton}
            onClick={() => onMove(a.id, 1)}
            disabled={i === areas.length - 1 || !docked}
            aria-label="later"
          >
            ↓
          </button>
        </div>
      ))}
    </div>
  );
}
