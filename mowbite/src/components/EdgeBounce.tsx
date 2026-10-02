'use client';

import {useEffect} from 'react';
import {isApp} from '@/lib/native';

// the page springs a little when a fast scroll hits the top or bottom, and gives way when pulled
// past the edge. only in the android app, chrome has its own stretch for that
function ownsScroll(el: Element | null): boolean {
  for (let e = el; e && e !== document.body; e = e.parentElement) {
    if (e.matches('svg, input, textarea, select, [role=slider], [data-no-swipe]')) return true;
    const y = getComputedStyle(e).overflowY;
    if ((y === 'auto' || y === 'scroll') && e.scrollHeight > e.clientHeight) return true;
  }
  return false;
}

export default function EdgeBounce() {
  useEffect(() => {
    const el = document.querySelector<HTMLElement>('[data-page]');
    if (!isApp() || !el || window.matchMedia('(prefers-reduced-motion: reduce)').matches) return;
    const root = document.documentElement;
    root.style.overscrollBehaviorY = 'none';
    const bottom = () => root.scrollHeight - window.innerHeight - 1;
    let anim: Animation | null = null;

    const spring = (from: number, kick: number) => {
      anim?.cancel();
      anim = el.animate(
        [
          {transform: `translateY(${from}px)`, easing: 'cubic-bezier(0.2, 0.7, 0.4, 1)'},
          {transform: `translateY(${kick}px)`, offset: 0.3, easing: 'cubic-bezier(0.4, 0, 0.3, 1)'},
          {transform: `translateY(${-kick * 0.12}px)`, offset: 0.7},
          {transform: 'translateY(0)'},
        ],
        {duration: 550},
      );
    };

    // fling into the edge
    let lastY = window.scrollY;
    let lastT = performance.now();
    let v = 0;
    let touching = false;
    const scroll = () => {
      const now = performance.now();
      const y = window.scrollY;
      if (now > lastT) v = 0.6 * ((y - lastY) / (now - lastT)) + 0.4 * v;
      lastY = y;
      lastT = now;
      if (touching || Math.abs(v) < 0.8) return;
      const amp = Math.min(44, Math.abs(v) * 12);
      if (y <= 0 && v < 0) spring(0, amp);
      else if (y >= bottom() && v > 0) spring(0, -amp);
      else return;
      v = 0;
    };

    // pulling past the edge with the finger
    let pull: {y: number; x: number; top: boolean; end: boolean; axis: 'x' | 'y' | null} | null = null;
    let shown = 0;
    const down = (e: TouchEvent) => {
      touching = true;
      pull = null;
      if (e.touches.length > 1 || ownsScroll(e.target as Element)) return;
      const t = e.touches[0];
      pull = {y: t.clientY, x: t.clientX, top: window.scrollY <= 0, end: window.scrollY >= bottom(), axis: null};
      shown = 0;
    };
    const move = (e: TouchEvent) => {
      if (!pull) return;
      const t = e.touches[0];
      const dy = t.clientY - pull.y;
      if (!pull.axis) {
        if (Math.max(Math.abs(dy), Math.abs(t.clientX - pull.x)) < 10) return;
        pull.axis = Math.abs(dy) > Math.abs(t.clientX - pull.x) ? 'y' : 'x';
      }
      if (pull.axis !== 'y' || !((pull.top && dy > 0) || (pull.end && dy < 0))) {
        if (shown) el.style.transform = '';
        shown = 0;
        return;
      }
      // harder the further it goes
      shown = Math.sign(dy) * 90 * (1 - Math.exp(-Math.abs(dy) / 260));
      anim?.cancel();
      el.style.transform = `translateY(${shown}px)`;
    };
    const up = () => {
      touching = false;
      pull = null;
      if (!shown) return;
      el.style.transform = '';
      spring(shown, 0);
      shown = 0;
    };

    window.addEventListener('scroll', scroll, {passive: true});
    window.addEventListener('touchstart', down, {passive: true});
    window.addEventListener('touchmove', move, {passive: true});
    window.addEventListener('touchend', up, {passive: true});
    window.addEventListener('touchcancel', up, {passive: true});
    return () => {
      root.style.overscrollBehaviorY = '';
      window.removeEventListener('scroll', scroll);
      window.removeEventListener('touchstart', down);
      window.removeEventListener('touchmove', move);
      window.removeEventListener('touchend', up);
      window.removeEventListener('touchcancel', up);
    };
  }, []);

  return null;
}
