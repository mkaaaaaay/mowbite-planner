import {imageryInfo, type CustomImagery, type ImagerySource} from '@/lib/imagery';
import {tr} from '@/lib/i18n';
import Link from 'next/link';
import styles from './MapView.module.css';

export const LAYERS = [
  {key: 'obstacle', label: 'Obstacles'},
  {key: 'nav', label: 'Navigation areas'},
  {key: 'order', label: 'Mowing order numbers'},
  {key: 'stripes', label: 'Mowing direction'},
  {key: 'track', label: 'Track'},
  {key: 'transit', label: 'Driving without blades'},
  {key: 'planDone', label: 'Mowed part of the plan'},
] as const;
export type Layer = (typeof LAYERS)[number]['key'];

// off until switched on, the others the other way round
const OFF_BY_DEFAULT: ReadonlySet<Layer> = new Set<Layer>(['planDone']);

// how the part of the plan still to mow is drawn: dots are pellets for the pac-man mower to eat
export const PLAN_STYLES = [
  {key: 'dashed', label: 'dashed'},
  {key: 'solid', label: 'solid'},
  {key: 'dots', label: 'pellets'},
] as const;
export type PlanStyle = (typeof PLAN_STYLES)[number]['key'];

// the menu keeps the layers switched away from their default
export const layerOn = (flipped: Set<Layer>, l: Layer) => flipped.has(l) === OFF_BY_DEFAULT.has(l);

// the buttons along the right edge of the map, the layer menu and the imagery bar
export default function MapControls({
  onZoom,
  showGrid,
  onToggleGrid,
  sources,
  source,
  imagerySettings,
  onSource,
  hidden,
  onToggleLayer,
  layersOpen,
  onLayersOpen,
  planStyle,
  reset,
  follow,
}: {
  // factor < 1 zooms in
  onZoom: (factor: number) => void;
  showGrid: boolean;
  onToggleGrid: () => void;
  sources: ImagerySource[];
  source: ImagerySource | null;
  imagerySettings: CustomImagery | undefined;
  onSource: (s: ImagerySource | null) => void;
  hidden: Set<Layer>;
  onToggleLayer: (l: Layer) => void;
  layersOpen: boolean;
  onLayersOpen: (open: boolean) => void;
  // only while there's a plan being mowed
  planStyle?: {value: PlanStyle; onChange: (s: PlanStyle) => void};
  // shown when zoomed or panned, follow: the view follows the mower
  reset: {follow: boolean; onReset: () => void} | null;
  // only while the view can follow the mower
  follow?: {on: boolean; onToggle: () => void};
}) {
  return (
    <>
      <div className={styles.zoomButtons}>
        <button onClick={() => onZoom(1 / 1.5)} aria-label="zoom in" title={tr('Zoom in (or mouse wheel / pinch)')}>
          +
        </button>
        <button onClick={() => onZoom(1.5)} aria-label="zoom out" title={tr('Zoom out')}>
          −
        </button>
        <button
          className={showGrid ? '' : styles.off}
          onClick={onToggleGrid}
          aria-label="toggle grid"
          title={tr('Show or hide the meter grid')}
        >
          #
        </button>
        {sources.length > 0 && (
          <button
            className={source ? '' : styles.off}
            onClick={() => onSource(source ? null : sources[0])}
            aria-label="toggle aerial imagery"
            title={tr('Aerial imagery (loads images of this area from the selected provider)')}
          >
            ◩
          </button>
        )}
        <button
          className={hidden.size ? styles.partly : ''}
          onClick={() => onLayersOpen(!layersOpen)}
          aria-label="layers"
          title={tr('Show or hide obstacles, numbers, track and more')}
        >
          <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinejoin="round">
            <path d="M12 3 3 8l9 5 9-5-9-5Z" />
            <path d="m3 12 9 5 9-5" />
            <path d="m3 16 9 5 9-5" />
          </svg>
        </button>
        <Link href="/settings" className={styles.linkButton} aria-label="settings" title={tr('Map colors, icons and aerial imagery')}>
          ⚙
        </Link>
        {follow && (
          <button
            className={follow.on ? '' : styles.off}
            onClick={follow.onToggle}
            aria-label="follow mower"
            title={follow.on ? tr('Following the mower, click to move the map freely') : tr('Follow the mower again')}
          >
            <svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round">
              <circle cx="12" cy="12" r="6" />
              <circle cx="12" cy="12" r="1.5" fill="currentColor" />
              <path d="M12 2v4M12 18v4M2 12h4M18 12h4" />
            </svg>
          </button>
        )}
        {reset && (
          <button
            onClick={reset.onReset}
            aria-label="reset zoom"
            title={reset.follow ? tr('Back to the default zoom') : tr('Fit the whole map')}
          >
            ⤢
          </button>
        )}
      </div>
      {layersOpen && (
        <div className={styles.layers}>
          {/* the plan's layers only while there's a mowing run to show them for */}
          {LAYERS.filter((l) => l.key !== 'planDone' || planStyle).map((l) => (
            <label key={l.key}>
              <input type="checkbox" checked={layerOn(hidden, l.key)} onChange={() => onToggleLayer(l.key)} />
              {tr(l.label)}
            </label>
          ))}
          {planStyle && (
            <label>
              {tr('Rest of the plan')}
              <select value={planStyle.value} onChange={(e) => planStyle.onChange(e.target.value as PlanStyle)}>
                {PLAN_STYLES.map((st) => (
                  <option key={st.key} value={st.key}>
                    {tr(st.label)}
                  </option>
                ))}
              </select>
            </label>
          )}
        </div>
      )}
      {source && (
        <div className={styles.imageryBar}>
          {sources.length > 1 && (
            <select value={source} onChange={(e) => onSource(e.target.value as ImagerySource)}>
              {sources.map((s) => (
                <option key={s} value={s}>
                  {tr(imageryInfo(s, imagerySettings).label)}
                </option>
              ))}
            </select>
          )}
          <span>© {imageryInfo(source, imagerySettings).attribution}</span>
        </div>
      )}
    </>
  );
}
