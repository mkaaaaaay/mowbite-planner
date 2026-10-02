import type {CapacitorConfig} from '@capacitor/cli';

// the android app: the same static build, talking to mowers in the local network over plain
// http/ws. served as http://localhost inside the app so that isn't blocked as mixed content
const config: CapacitorConfig = {
  appId: 'io.github.mkaaaaaay.mowbite',
  appName: 'MowBite',
  webDir: 'out',
  server: {
    androidScheme: 'http',
    cleartext: true,
  },
  backgroundColor: '#101114',
  plugins: {
    // edge to edge like android 15 wants it, the page pads itself with env(safe-area-inset-*).
    // light icons in the status bar on the dark app
    SystemBars: {
      insetsHandling: 'native',
      initialViewportFitValueHint: 'cover',
      style: 'DARK',
    },
  },
};

export default config;
