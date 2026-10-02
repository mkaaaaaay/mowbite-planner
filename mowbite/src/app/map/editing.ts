import type {MowerMap} from '@/hooks/useMowerMap';
import {fmt, tr} from '@/lib/i18n';

export type Area = MowerMap['areas'][number];
export type AreaProperties = Area['properties'];
// change the selected area's properties, undoable = one undo step for this change
export type UpdateArea = (patch: Partial<AreaProperties>, undoable?: boolean) => void;

// area settings only newer openmower versions keep, older ones drop them when saving
export const NEW_AREA_SETTINGS = ['mowable', 'angle_min', 'angle_max'] as const;

export const AREA_TYPES = [
  {value: 'mow', label: 'Mowing area', hint: 'driven on and mowed'},
  {value: 'nav', label: 'Navigation area', hint: 'driven on but not mowed, e.g. a path between two lawns'},
  {value: 'obstacle', label: 'Obstacle', hint: 'no-go zone, keep it inside a mowing area'},
  {value: 'draft', label: 'Draft', hint: 'ignored by the mower'},
];

export const DEG = Math.PI / 180;

// wraps into -180..180
export function normDeg(d: number) {
  return ((((d + 180) % 360) + 360) % 360) - 180;
}

export type Override = 'outline_count' | 'outline_overlap_count' | 'outline_offset';

// the limits of the mower's own settings (MowerLogic.cfg). the ones per area go to the planner unchecked
const LIMITS: Record<Override, {min: number; max: number; whole: boolean}> = {
  outline_count: {min: 0, max: 255, whole: true},
  outline_overlap_count: {min: 0, max: 255, whole: true},
  outline_offset: {min: -1, max: 1, whole: false},
};

export function overrideError(key: Override, raw: string): string | null {
  if (raw.trim() === '') return null;
  const v = Number(raw);
  const {min, max, whole} = LIMITS[key];
  if (Number.isFinite(v) && v >= min && v <= max && (!whole || Number.isInteger(v))) return null;
  return whole
    ? tr('a whole number from {min} to {max}', {min, max})
    : tr('from {min} to {max} m', {min: fmt(min), max: fmt(max)});
}
