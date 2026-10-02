import {useEffect, useRef, useState} from 'react';

interface Position {
  x: number;
  y: number;
}

// no speed on the wire, so estimate m/s from pose updates (smoothed, gps jitter is noisy)
export function useComputedSpeed(position: Position | null | undefined, halfLifeMs = 400): number {
  const [speed, setSpeed] = useState(0);
  const lastRef = useRef<{x: number; y: number; t: number} | null>(null);
  const smoothedRef = useRef(0);

  useEffect(() => {
    if (!position) return;
    const now = performance.now();
    const last = lastRef.current;

    if (last) {
      const dt = (now - last.t) / 1000;
      if (dt > 0.05) {
        const dist = Math.hypot(position.x - last.x, position.y - last.y);
        const instantSpeed = dist / dt;
        const alpha = 1 - Math.pow(0.5, dt / (halfLifeMs / 1000));
        smoothedRef.current += (instantSpeed - smoothedRef.current) * alpha;
        setSpeed(smoothedRef.current);
        lastRef.current = {x: position.x, y: position.y, t: now};
      }
    } else {
      lastRef.current = {x: position.x, y: position.y, t: now};
    }
  }, [position, halfLifeMs]);

  return speed;
}
