import type {Point} from '@/hooks/useMowerMap';
import InfoTip from '@/components/InfoTip';
import {polygonArea} from '@/lib/geometry';
import {fmt, tr} from '@/lib/i18n';
import type {Area} from './editing';
import styles from './page.module.css';

// the small panels for the steps that take over the editor for a moment

export function SplitPanel({
  preview,
  cutShape,
  onCutShape,
  points,
  onApply,
  onRemoveLast,
  onCancel,
}: {
  preview: Point[][] | null;
  cutShape: boolean;
  onCutShape: (on: boolean) => void;
  points: number;
  onApply: () => void;
  onRemoveLast: () => void;
  onCancel: () => void;
}) {
  const pieceClass = [styles.pieceA, styles.pieceB, styles.pieceC];
  return (
    <div className={styles.splitBox}>
      <label className={styles.toggle}>
        <input type="checkbox" checked={cutShape} onChange={(e) => onCutShape(e.target.checked)} />
        {tr('Cut out a shape')}
        <InfoTip>
          {tr("Off: a line across the area cuts it in two.")}
          <br />
          {tr("On: click the corners of a shape inside the area. It becomes its own area with the same settings and the rest leaves it out. Since areas can't have holes, the rest is split in two by the shortest straight cut through the shape. The mower drives across the cut normally.")}
          <br />
          {tr('Already drawn the inner area? Select it, the note there cuts it out in one click.')}
        </InfoTip>
      </label>
      <p className={styles.dim}>
        {cutShape
          ? tr("Click the corners of a shape inside the area. It becomes its own area, the rest is split in two through it (areas can't have holes).")
          : tr('Click points for a cut line across the area, start and end outside. Drag points to move them.')}
      </p>
      {preview ? (
        <p>
          {preview.map((piece, i) => (
            <span key={i}>
              {i > 0 && (i === preview.length - 1 ? ` ${tr('and')} ` : ', ')}
              <span className={pieceClass[i]}>{fmt(polygonArea(piece), 1)} m²</span>
            </span>
          ))}
        </p>
      ) : (
        points >= (cutShape ? 3 : 2) && (
          <p className={styles.dim}>
            {cutShape
              ? tr("The shape has to lie fully inside the area and mustn't cross itself.")
              : tr("The line doesn't cut through the area yet.")}
          </p>
        )
      )}
      <div className={styles.inlineRow}>
        <button className={styles.pillButton} onClick={onApply} disabled={!preview}>
          {tr('Apply split')}
        </button>
        <button className={styles.pillButton} onClick={onRemoveLast} disabled={!points}>
          {tr('Remove last point')}
        </button>
        <button className={styles.pillButton} onClick={onCancel}>
          {tr('Cancel')}
        </button>
      </div>
    </div>
  );
}

export function MergePanel({
  area,
  other,
  merged,
  onApply,
  onCancel,
}: {
  area: Area;
  other: Area | null;
  merged: {outline: Point[]; holesFilled: number} | null;
  onApply: () => void;
  onCancel: () => void;
}) {
  return (
    <div className={styles.splitBox}>
      <p className={styles.dim}>
        {tr("Click the area to merge into {name}. The result keeps its name, type and settings.", {name: area.properties.name || tr('this one')})}
      </p>
      {other && merged && (
        <p>
          {area.properties.name || tr('unnamed')} + {other.properties.name || tr('unnamed')} ={' '}
          <span className={styles.pieceA}>{fmt(polygonArea(merged.outline), 1)} m²</span>
          {merged.holesFilled > 0 && tr(', the gap enclosed between them gets filled in')}
          {other.properties.type !== area.properties.type &&
            tr(', careful: {name} is a different type', {name: other.properties.name || tr('it')})}
        </p>
      )}
      {other && !merged && <p className={styles.error}>{tr("These two don't touch, there'd be two separate pieces.")}</p>}
      <div className={styles.inlineRow}>
        <button className={styles.pillButton} onClick={onApply} disabled={!merged}>
          {tr('Apply merge')}
        </button>
        <button className={styles.pillButton} onClick={onCancel}>
          {tr('Cancel')}
        </button>
      </div>
    </div>
  );
}

export function DrawPanel({points, onFinish, onCancel}: {points: number; onFinish: () => void; onCancel: () => void}) {
  return (
    <div className={styles.inlineRow}>
      <span className={styles.dim}>{tr('Click points to draw the outline ({n} so far), drag them to move', {n: points})}</span>
      <button className={styles.pillButton} onClick={onFinish} disabled={points < 3}>
        {tr('Finish')}
      </button>
      <button className={styles.pillButton} onClick={onCancel}>
        {tr('Cancel')}
      </button>
    </div>
  );
}

// the reduce-points slider, cm is how far the new outline may be off
export function SimplifyPanel({
  cm,
  from,
  to,
  onChange,
  onApply,
  onCancel,
}: {
  cm: number;
  from: number;
  to: number;
  onChange: (cm: number) => void;
  onApply: () => void;
  onCancel: () => void;
}) {
  return (
    <div className={styles.inlineRow}>
      <input type="range" min={0} max={20} value={cm} onChange={(e) => onChange(Number(e.target.value))} />
      <span className={styles.dim}>
        {cm === 0 ? tr('all points') : tr('max. {n} cm off', {n: cm})} · {from} → {tr('{n} points', {n: to})}
      </span>
      <button className={styles.pillButton} onClick={onApply}>
        {tr('Apply')}
      </button>
      <button className={styles.pillButton} onClick={onCancel}>
        {tr('Cancel')}
      </button>
    </div>
  );
}

// a backup (or a map from a file) shown instead of the editor
export function RestorePanel({
  areas,
  backedUp,
  docked,
  restoring,
  confirm,
  error,
  onRestore,
  onDisarm,
  onClose,
}: {
  areas: number;
  // the mower's map is backed up before restoring (only when MowBite runs as its container)
  backedUp: boolean;
  docked: boolean;
  restoring: boolean;
  confirm: boolean;
  error: string | null;
  onRestore: () => void;
  onDisarm: () => void;
  onClose: () => void;
}) {
  return (
    <div className={styles.splitBox}>
      <p>{tr('{n} areas', {n: areas})}</p>
      <p className={styles.dim}>
        {backedUp
          ? tr('Restoring replaces the map on the mower with this one. The current map is backed up first, so this can be undone.')
          : tr("Restoring replaces the map on the mower with this one. Backups don't work here, download the current map first if you might want it back.")}
      </p>
      {!docked && <p className={styles.error}>{tr('Only possible while the mower is idle in the dock.')}</p>}
      <div className={styles.inlineRow}>
        <button
          className={[styles.pillButton, styles.danger].join(' ')}
          disabled={!docked || restoring}
          onClick={onRestore}
          onBlur={onDisarm}
        >
          {restoring ? tr('restoring…') : confirm ? tr('Really restore?') : tr('Restore')}
        </button>
        <button className={styles.pillButton} onClick={onClose}>
          {tr('Close')}
        </button>
      </div>
      {error && <p className={styles.error}>{error}</p>}
    </div>
  );
}
