import {MergeIcon, ScissorsIcon, SimplifyIcon, TrashIcon} from '@/components/icons';
import InfoTip from '@/components/InfoTip';
import {polygonArea} from '@/lib/geometry';
import {fmt, tr} from '@/lib/i18n';
import {useAreaProperties} from '@/lib/areaProps';
import {AREA_TYPES, type Area, type UpdateArea} from './editing';
import styles from './page.module.css';

// name, type and active switch of the selected area, and what can be done with it
export default function AreaCard({
  area,
  enclosing,
  onCutOut,
  showTools,
  confirmDelete,
  remember,
  update,
  onSplit,
  onMerge,
  onSimplify,
  onDelete,
  onDeleteBlur,
}: {
  area: Area;
  enclosing?: Area;
  // cuts this area out of the enclosing one, null when it doesn't lie fully inside
  onCutOut: (() => void) | null;
  showTools: boolean;
  confirmDelete: boolean;
  remember: () => void;
  update: UpdateArea;
  onSplit: () => void;
  onMerge: () => void;
  onSimplify: () => void;
  onDelete: () => void;
  onDeleteBlur: () => void;
}) {
  const type = area.properties.type ?? 'draft';
  const supported = useAreaProperties();
  const tool = [styles.pillButton, styles.tool].join(' ');
  const outer = enclosing && {name: enclosing.properties.name || tr('unnamed')};
  const outerMows = enclosing?.properties.type === 'mow' && enclosing.properties.mowable !== false;
  const inactive = area.properties.active === false;
  const skipped = inactive || type === 'nav' || area.properties.mowable === false;
  // a mower that knows mow_around also keeps the blade off over a don't mow area, nothing to warn about then
  const bladeOff = supported.has('mow_around') && type === 'mow' && !inactive && area.properties.mowable === false;
  const nested =
    outer &&
    !bladeOff &&
    (outerMows && skipped
      ? tr('Lies inside "{name}", so it gets mowed and driven on anyway.', outer)
      : outerMows
        ? tr('Lies inside "{name}" and gets mowed with it as well.', outer)
        : inactive
          ? tr('Lies inside "{name}", so the mower still drives here although this area is inactive.', outer)
          : null);
  return (
    <div className={styles.areaEditor}>
      <div className={styles.areaHead}>
        <input
          className={styles.nameInput}
          value={area.properties.name ?? ''}
          placeholder={tr('unnamed')}
          // one undo step per rename, not per keystroke
          onFocus={remember}
          onChange={(e) => update({name: e.target.value}, false)}
        />
        <select className={styles.typeSelect} value={type} onChange={(e) => update({type: e.target.value})}>
          {AREA_TYPES.map((t) => (
            <option key={t.value} value={t.value}>
              {tr(t.label)}
            </option>
          ))}
        </select>
      </div>
      <div className={styles.areaMeta}>
        <label className={styles.toggle}>
          <input
            type="checkbox"
            checked={area.properties.active !== false}
            onChange={() => update({active: area.properties.active === false})}
          />
          {tr('active')}
          <InfoTip>
            {tr('Ignored by the mower and not drivable. Careful: if the mower stands on an inactive area, it gets stuck.')}
          </InfoTip>
        </label>
        {type === 'mow' && area.properties.active !== false && supported.has('mowable') && (
          <label className={styles.toggle}>
            <input
              type="checkbox"
              checked={area.properties.mowable === false}
              onChange={() => update({mowable: area.properties.mowable === false ? undefined : false})}
            />
            {tr("don't mow")}
            <InfoTip>
              {tr('Not mowed, but the mower may drive across it (unlike inactive).')}
            </InfoTip>
          </label>
        )}
        {type === 'mow' && area.properties.active !== false && area.properties.mowable === false && supported.has('mow_around') && (
          <label className={styles.toggle}>
            <input
              type="checkbox"
              checked={area.properties.mow_around === true}
              onChange={() => update({mow_around: area.properties.mow_around ? undefined : true})}
            />
            {tr('mow around it')}
            <InfoTip>
              {tr(
                'The mowing areas it lies in end their lanes at its edge and go around it once. Otherwise the lanes go across it with the blade off. It stays drivable either way.',
              )}
            </InfoTip>
          </label>
        )}
        <span className={styles.dim}>
          {area.properties.active === false
            ? tr('inactive: not driven on and not mowed')
            : type === 'mow' && area.properties.mowable === false
              ? tr('driven on, but not mowed')
              : tr(AREA_TYPES.find((t) => t.value === type)?.hint ?? '')}
          {' · '}
          {fmt(polygonArea(area.outline), 1)} m²
        </span>
      </div>
      {nested && (
        <div className={styles.warning}>
          {nested}{' '}
          {onCutOut
            ? tr("To leave it out, cut it out of \"{name}\" (it gets split in two, areas can't have holes). Or make this an obstacle so the mower never drives here.", outer)
            : tr("It doesn't lie fully inside, to leave it out cut \"{name}\" by hand with Split area.", outer)}
          {onCutOut && (
            <div className={styles.inlineRow}>
              <button className={styles.pillButton} onClick={onCutOut}>
                <ScissorsIcon size={16} />
                {tr('Cut out of "{name}"', outer)}
              </button>
            </div>
          )}
        </div>
      )}
      {showTools && (
        <div className={styles.toolbar}>
          <span className={styles.toolLabel}>
            {tr('Edit')}
            <InfoTip>
              <b>{tr('Split area')}:</b>{' '}
              {tr('Cuts the area in two along a line, or cuts a shape out of it. All parts keep type and settings.')}
              <br />
              <b>{tr('Merge')}:</b>{' '}
              {tr('Joins this area with another one you click. They need to overlap or touch.')}
              <br />
              <b>{tr('Reduce points')}:</b>{' '}
              {tr('Recorded outlines have a point every few cm. This removes the ones that barely change the shape, the slider sets how far the outline may move.')}
            </InfoTip>
          </span>
          <button className={tool} onClick={onSplit}>
            <ScissorsIcon size={16} />
            {tr('Split area')}
          </button>
          <button className={tool} onClick={onMerge}>
            <MergeIcon size={16} />
            {tr('Merge')}
          </button>
          <button className={tool} onClick={onSimplify}>
            <SimplifyIcon size={16} />
            {tr('Reduce points')}
          </button>
          <button className={[tool, styles.danger].join(' ')} onClick={onDelete} onBlur={onDeleteBlur}>
            <TrashIcon size={16} />
            {confirmDelete ? tr('Really delete?') : tr('Delete area')}
          </button>
        </div>
      )}
    </div>
  );
}
