'use client';

import {isApp} from '@/lib/native';
import {useEffect} from 'react';

// The container's web server sends no cache headers, so a phone's browser may keep showing an older build for a
// while after an update. The build on the mower says its version in version.json: when that differs from the one
// running here, the page is fetched again past the cache and reloaded. Once per version and page, so it can't loop.
export default function FreshPage() {
  useEffect(() => {
    // the android app brings its own build
    if (isApp()) return;
    const check = async () => {
      try {
        const res = await fetch('/version.json', {cache: 'no-store'});
        if (!res.ok) return;
        const {version} = (await res.json()) as {version?: string};
        if (!version || version === process.env.APP_VERSION) return;
        // per page, the browser keeps each one on its own
        const key = `reloadedFor:${location.pathname}`;
        if (sessionStorage.getItem(key) === version) return;
        sessionStorage.setItem(key, version);
        await fetch(location.href, {cache: 'reload'}).catch(() => {});
        location.reload();
      } catch {}
    };
    void check();
    const onVisible = () => document.visibilityState === 'visible' && void check();
    document.addEventListener('visibilitychange', onVisible);
    return () => document.removeEventListener('visibilitychange', onVisible);
  }, []);
  return null;
}
