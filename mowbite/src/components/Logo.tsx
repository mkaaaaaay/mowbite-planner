'use client';

import {useId, useRef, useState} from 'react';
import styles from './Logo.module.css';

const OPEN = 'M24 32 L39.6 23 A18 18 0 1 0 39.6 41 Z';
const SHUT = 'M24 32 L42 31.9 A18 18 0 1 0 42 32.1 Z';
const CHOMP_MS = 3000;

// a lawn eater chomping its way through the grass. tap it and it does
// chomp: keep the mouth going. bare: just the eater, without its own grass (when it mows the real one). children:
// drawn on the eater, in its 64 box
export default function LogoMark({
  size = 28,
  className,
  chomp,
  bare,
  children,
}: {
  size?: number;
  className?: string;
  chomp?: boolean;
  bare?: boolean;
  children?: React.ReactNode;
}) {
  // own id per copy, a gradient inside a hidden svg (the nav's on phones) would paint nothing
  const grad = useId();
  const [chomping, setChomping] = useState(false);
  const timer = useRef<ReturnType<typeof setTimeout>>(undefined);

  const startChomp = () => {
    if (chomping || window.matchMedia('(prefers-reduced-motion: reduce)').matches) return;
    setChomping(true);
    clearTimeout(timer.current);
    timer.current = setTimeout(() => setChomping(false), CHOMP_MS);
  };

  return (
    <svg
      width={size}
      height={size}
      viewBox="0 0 64 64"
      aria-hidden="true"
      className={[className, chomping ? styles.chomping : ''].filter(Boolean).join(' ')}
      onClick={startChomp}
    >
      <defs>
        <linearGradient id={grad} x1="0" y1="0" x2="0" y2="1">
          <stop offset="0" stopColor="#8fe07c" />
          <stop offset="1" stopColor="#35a64f" />
        </linearGradient>
      </defs>
      {!bare && (
        <g fill="none" stroke="#5cc36a" strokeWidth="2.6" strokeLinecap="round">
          <path className={styles.tuft1} d="M45 45 Q46 38 44 33 M48.5 45 Q48.5 36 51 30 M52 45 Q52 39 55 35" />
          <path className={styles.tuft2} d="M56 45 Q57 41 55.5 38 M59 45 Q59.5 40 61.5 37" opacity="0.7" />
        </g>
      )}
      <g className={styles.body}>
        <path d={OPEN} fill={`url(#${grad})`}>
          {(chomping || chomp) && <animate attributeName="d" values={`${OPEN};${SHUT};${OPEN}`} dur="0.28s" repeatCount="indefinite" />}
        </path>
        <circle cx="26" cy="21.5" r="2.4" fill="#10261a" />
        {children}
      </g>
      {!bare && <path d="M8 50 H40" stroke="#35a64f" strokeWidth="2.6" strokeLinecap="round" opacity="0.5" />}
    </svg>
  );
}

export function TitleMark() {
  return <LogoMark size={28} className={styles.titleMark} />;
}
