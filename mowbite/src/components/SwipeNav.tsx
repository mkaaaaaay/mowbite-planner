'use client';

import {usePathname, useRouter} from 'next/navigation';
import {lazy, Suspense, useEffect, useLayoutEffect, useRef, useState} from 'react';
import {createPortal} from 'react-dom';
import layout from '@/app/layout.module.css';
import {swipeEnabled} from '@/lib/swipe';

// the pages in the order of the tab bar
const ORDER = ['/', '/map', '/sensors', '/schedule', '/activity', '/settings'];
// things that want horizontal swipes themselves: the map, the stick, sliders, fields, chip rows
function claimsSwipe(el: Element | null): boolean {
  for (let e = el; e && e !== document.body; e = e.parentElement) {
    if (e.matches('svg, input, textarea, select, [role=slider], [data-no-swipe]')) return true;
    const x = getComputedStyle(e).overflowX;
    if ((x === 'auto' || x === 'scroll') && e.scrollWidth > e.clientWidth) return true;
  }
  return false;
}

// the neighbour page is rendered for real while swiping, so it's there even on the first swipe
const LOAD: Record<string, () => Promise<{default: React.ComponentType}>> = {
  '/': () => import('@/app/page'),
  '/map': () => import('@/app/map/page'),
  '/sensors': () => import('@/app/sensors/page'),
  '/schedule': () => import('@/app/schedule/page'),
  '/activity': () => import('@/app/activity/page'),
  '/settings': () => import('@/app/settings/page'),
};
const PAGES = Object.fromEntries(Object.entries(LOAD).map(([p, load]) => [p, lazy(load)]));

const calm = () => window.matchMedia('(prefers-reduced-motion: reduce)').matches;
const page = () => document.querySelector<HTMLElement>('[data-page]');
const EASE = 'cubic-bezier(0.2, 0.8, 0.2, 1)';

// set when a swipe navigates, until the new page is rendered
let entering = false;

// optional (settings, per device): swipe left/right between the pages on a phone.
// the page follows the finger with the next one right beside it, like turning pages
export default function SwipeNav() {
  const router = useRouter();
  const path = usePathname().replace(/(.)\/$/, '$1');
  // the page shown beside the current one, and on which side
  const [peek, setPeek] = useState<{path: string; side: number} | null>(null);
  const box = useRef<HTMLDivElement>(null);

  // the new page is rendered: put it in place under the peek. it still fetches its data, so the
  // peek stays on top until the page stops changing, then fades out. no empty flash in between
  useLayoutEffect(() => {
    const el = page();
    if (!entering || !el) return;
    entering = false;
    el.style.transition = 'none';
    el.style.translate = '';
    el.style.opacity = '';
    let quiet: ReturnType<typeof setTimeout> | undefined;
    let gone: ReturnType<typeof setTimeout> | undefined;
    const reveal = () => {
      watch.disconnect();
      clearTimeout(quiet);
      clearTimeout(latest);
      const b = box.current;
      if (b) {
        b.style.transition = 'opacity 0.2s ease-out';
        b.style.opacity = '0';
      }
      gone = setTimeout(() => setPeek(null), 220);
    };
    const settled = () => {
      clearTimeout(quiet);
      quiet = setTimeout(reveal, 200);
    };
    // attributes change all the time on the map (the mower moving), only new content counts
    const watch = new MutationObserver(settled);
    watch.observe(el, {childList: true, subtree: true, characterData: true});
    settled();
    // don't wait forever on a page that keeps changing
    const latest = setTimeout(reveal, 1500);
    return () => {
      watch.disconnect();
      clearTimeout(quiet);
      clearTimeout(latest);
      clearTimeout(gone);
      // left again before it was done: don't leave the peek behind
      setTimeout(() => setPeek(null), 0);
    };
  }, [path]);

  useEffect(() => {
    // not while recording, the stick is in use there
    if (path === '/record') return;
    const i = ORDER.indexOf(path);
    const neighbour = (dx: number) => {
      const n = i === -1 ? -1 : i + (dx < 0 ? 1 : -1);
      return n >= 0 && n < ORDER.length ? ORDER[n] : null;
    };
    let drag: {x: number; y: number; t: number; axis: 'x' | 'y' | null; el: HTMLElement | null; side: number} | null =
      null;
    let dx = 0;
    const w = () => window.innerWidth;

    // back to where it was
    const settle = (el: HTMLElement | null, side: number) => {
      const t = `translate 0.3s ${EASE}, opacity 0.2s`;
      if (el) {
        el.style.transition = t;
        el.style.translate = '';
        el.style.opacity = '';
      }
      const b = box.current;
      if (b && side) {
        b.style.transition = t;
        b.style.translate = `${side * w()}px 0`;
      }
      setTimeout(() => setPeek(null), 320);
    };

    const down = (e: TouchEvent) => {
      const t = e.touches[0];
      drag = null;
      // the screen edges are android's back gesture
      if (!swipeEnabled() || e.touches.length > 1 || t.clientX < 24 || t.clientX > w() - 24) return;
      if (claimsSwipe(e.target as Element)) return;
      drag = {x: t.clientX, y: t.clientY, t: e.timeStamp, axis: null, el: page(), side: 0};
      dx = 0;
      // have the neighbours' code ready before the finger moves
      ORDER.forEach((p, n) => Math.abs(n - i) === 1 && void LOAD[p]());
    };
    const move = (e: TouchEvent) => {
      if (!drag) return;
      const t = e.touches[0];
      dx = t.clientX - drag.x;
      const dy = t.clientY - drag.y;
      if (!drag.axis) {
        if (Math.max(Math.abs(dx), Math.abs(dy)) < 10) return;
        drag.axis = Math.abs(dx) > Math.abs(dy) * 1.5 ? 'x' : 'y';
        if (drag.axis === 'x') ORDER.forEach((p, n) => Math.abs(n - i) === 1 && router.prefetch(p));
      }
      if (drag.axis !== 'x') return;
      e.preventDefault();
      if (!drag.el || calm()) return;
      const next = neighbour(dx);
      // the next page sits right of the current one, the previous left of it
      const side = dx < 0 ? 1 : -1;
      if (drag.side !== side) {
        drag.side = side;
        setPeek(next ? {path: next, side} : null);
      }
      // at the first/last page it only gives a little
      const shown = next ? dx : dx * 0.25;
      drag.el.style.transition = 'none';
      drag.el.style.translate = `${shown}px 0`;
      const b = box.current;
      if (b && next) {
        b.style.transition = 'none';
        b.style.translate = `${shown + side * w()}px 0`;
      }
    };
    const up = (e: TouchEvent) => {
      const d = drag;
      drag = null;
      if (!d || d.axis !== 'x') return;
      const next = neighbour(dx);
      const quick = e.timeStamp - d.t < 400;
      const go = next && (Math.abs(dx) > w() * 0.3 || (quick && Math.abs(dx) > 60));
      if (!go) {
        settle(d.el, d.side);
        return;
      }
      const dir = dx < 0 ? 1 : -1;
      if (!d.el || calm()) {
        router.push(next);
        return;
      }
      // the rest of the way, quicker the further it already is
      const ms = Math.round(120 + 180 * (1 - Math.min(Math.abs(dx) / w(), 1)));
      const el = d.el;
      el.style.transition = `translate ${ms}ms ${EASE}`;
      el.style.translate = `${-dir * w()}px 0`;
      const b = box.current;
      if (b) {
        b.style.transition = `translate ${ms}ms ${EASE}`;
        b.style.translate = '0px 0';
      }
      setTimeout(() => {
        entering = true;
        router.push(next);
        // in case the page never changes, don't leave it off screen
        setTimeout(() => {
          if (!entering) return;
          entering = false;
          settle(el, 0);
        }, 2000);
      }, ms);
    };
    const cancel = () => {
      if (drag?.axis === 'x') settle(drag.el, drag.side);
      drag = null;
    };
    window.addEventListener('touchstart', down, {passive: true});
    // not passive: a sideways drag must not also scroll the page
    window.addEventListener('touchmove', move, {passive: false});
    window.addEventListener('touchend', up, {passive: true});
    window.addEventListener('touchcancel', cancel, {passive: true});
    return () => {
      window.removeEventListener('touchstart', down);
      window.removeEventListener('touchmove', move);
      window.removeEventListener('touchend', up);
      window.removeEventListener('touchcancel', cancel);
    };
  }, [path, router]);

  if (!peek) return null;
  const Peek = PAGES[peek.path];
  return createPortal(
    <div
      ref={box}
      aria-hidden
      inert
      style={{
        position: 'fixed',
        inset: 0,
        overflow: 'hidden',
        pointerEvents: 'none',
        background: 'var(--background)',
        zIndex: 1,
        translate: `${peek.side * 100}vw 0`,
      }}
    >
      <div className={layout.content}>
        <Suspense fallback={null}>
          <Peek />
        </Suspense>
      </div>
    </div>,
    document.body,
  );
}
