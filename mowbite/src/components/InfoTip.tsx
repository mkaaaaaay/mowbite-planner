'use client';

import {useEffect, useRef, useState} from 'react';
import {createPortal} from 'react-dom';
import styles from './InfoTip.module.css';

// wide enough for a few sentences, narrower on small phones
const MAX_WIDTH = 320;

// small (i) with an explanation, hover on desktop, tap on touch (title tooltips don't work there).
// rendered into body, the panels scroll and use backdrop-filter which would clip it
export default function InfoTip({children}: {children: React.ReactNode}) {
  const [pos, setPos] = useState<{left: number; top: number; width: number} | null>(null);
  const ref = useRef<HTMLButtonElement>(null);

  const show = () => {
    const r = ref.current?.getBoundingClientRect();
    if (!r) return;
    const width = Math.min(MAX_WIDTH, window.innerWidth - 16);
    setPos({left: Math.min(window.innerWidth - width - 8, Math.max(8, r.left + r.width / 2 - width / 2)), top: r.bottom + 6, width});
  };

  useEffect(() => {
    if (!pos) return;
    const close = (e: Event) => {
      if (e.target !== ref.current) setPos(null);
    };
    window.addEventListener('pointerdown', close);
    window.addEventListener('scroll', close, true);
    return () => {
      window.removeEventListener('pointerdown', close);
      window.removeEventListener('scroll', close, true);
    };
  }, [pos]);

  return (
    <>
      <button
        ref={ref}
        type="button"
        className={styles.icon}
        aria-label="more info"
        onPointerEnter={(e) => e.pointerType === 'mouse' && show()}
        onPointerLeave={(e) => e.pointerType === 'mouse' && setPos(null)}
        onClick={(e) => {
          e.preventDefault();
          e.stopPropagation();
          if (pos) setPos(null);
          else show();
        }}
      >
        i
      </button>
      {pos &&
        createPortal(
          <span className={styles.bubble} style={{left: pos.left, top: pos.top, width: pos.width}}>
            {children}
          </span>,
          document.body,
        )}
    </>
  );
}
