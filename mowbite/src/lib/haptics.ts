import {isApp} from './native';

// a short tick under the finger, e.g. when a point on the map is picked up. the app uses the phone's haptics, a
// browser vibrates where it can (chrome on android, iphones have no vibration in the browser)
export function tick() {
  if (isApp()) {
    void import('@capacitor/haptics').then(({Haptics, ImpactStyle}) => Haptics.impact({style: ImpactStyle.Light})).catch(() => {});
    return;
  }
  try {
    navigator.vibrate?.(10);
  } catch {}
}
