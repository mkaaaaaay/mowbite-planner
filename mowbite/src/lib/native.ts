// true inside the android app (capacitor puts its bridge on window). the app has no mower of its
// own, it talks to the ones the user added, over the local network
export function isApp(): boolean {
  return typeof window !== 'undefined' && !!(window as {Capacitor?: {isNativePlatform?: () => boolean}}).Capacitor?.isNativePlatform?.();
}
