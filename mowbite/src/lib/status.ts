import {tr} from './i18n';

export type StatusColor = 'success' | 'warning' | 'error' | 'accent';

export function stateColor(state: string | undefined): StatusColor {
  switch (state) {
    case 'MOWING':
    case 'DOCKED':
      return 'success';
    case 'PAUSED':
    case 'DOCKING':
    case 'UNDOCKING':
      return 'warning';
    case 'ERROR':
      return 'error';
    default:
      return 'accent';
  }
}

export function batteryColor(percent: number): 'success' | 'warning' | 'error' {
  if (percent > 50) return 'success';
  if (percent > 20) return 'warning';
  return 'error';
}

// idle in the station. is_charging drops when the battery is full, so the charge voltage counts too
export function isDocked(state: {current_state: string; is_charging: number} | null | undefined, vCharge?: string) {
  return state?.current_state === 'IDLE' && (!!state.is_charging || Number(vCharge) > 20);
}

export function statusText(
  state: {current_state: string; emergency: number; is_charging: number} | null | undefined,
  docked: boolean,
  chargeState?: string,
): string {
  if (!state) return '';
  if (state.emergency) return tr('Emergency');
  if (docked) return chargeState === 'Done' ? tr('Docked · charged') : tr('Docked · charging');
  const names: Record<string, string> = {
    IDLE: 'Idle',
    MOWING: 'Mowing',
    DOCKING: 'Returning to dock',
    UNDOCKING: 'Leaving the dock',
    AREA_RECORDING: 'Recording an area',
    PAUSED: 'Paused',
  };
  const name = names[state.current_state];
  if (name) return tr(name);
  return state.current_state.toLowerCase().replace(/_/g, ' ');
}

// how good the gps position is. The receiver doesn't report fix or float, so it's told by its estimated accuracy (m):
// a few cm is an rtk fix, up to the mower's limit (mower_logic/max_position_accuracy, 0.2 m by default) still
// usable, worse the mower ignores it. 999 and more is no fix at all (xbot_positioning)
export type GpsQuality = 'none' | 'fix' | 'float' | 'poor';
export function gpsQuality(accuracy: number | undefined, limit = 0.2): GpsQuality {
  if (accuracy === undefined || accuracy >= 999) return 'none';
  if (accuracy <= 0.05) return 'fix';
  return accuracy <= limit ? 'float' : 'poor';
}

export const GPS_QUALITY_LABEL: Record<GpsQuality, string> = {none: 'no fix', fix: 'RTK fix', float: 'float', poor: 'too inaccurate'};

