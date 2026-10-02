'use client';

import {useEffect, useRef, useState} from 'react';

export interface Pose {
  x: number;
  y: number;
  heading: number;
}

const JUMP = 5; // m, anything further just snaps

function wrapAngle(a: number) {
  return ((((a + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)) - Math.PI;
}

// Poses arrive a few times a second. Easing towards each new one looks like stop and go, so this plays
// them back one update interval late and moves at constant speed between the last two instead.
export function useEasedPose(target: Pose): Pose {
  const [pose, setPose] = useState(target);
  const samples = useRef<{t: number; pose: Pose}[]>([]);
  const interval = useRef(200);

  useEffect(() => {
    const now = performance.now();
    const list = samples.current;
    const last = list[list.length - 1];
    if (last && last.pose.x === target.x && last.pose.y === target.y && last.pose.heading === target.heading) return;
    if (last) {
      const dt = now - last.t;
      if (Math.hypot(target.x - last.pose.x, target.y - last.pose.y) > JUMP || dt > 3000) list.length = 0;
      else interval.current = interval.current * 0.8 + Math.min(1500, Math.max(50, dt)) * 0.2;
    }
    list.push({t: now, pose: target});
    if (list.length > 10) list.shift();
  }, [target]);

  useEffect(() => {
    let frame = 0;
    const tick = () => {
      const list = samples.current;
      if (list.length) {
        const at = performance.now() - interval.current;
        let next: Pose = list[list.length - 1].pose;
        if (list.length > 1 && at < list[list.length - 1].t) {
          let i = list.length - 2;
          while (i > 0 && list[i].t > at) i--;
          const a = list[i];
          const b = list[i + 1];
          const f = Math.min(1, Math.max(0, (at - a.t) / (b.t - a.t || 1)));
          next = {
            x: a.pose.x + (b.pose.x - a.pose.x) * f,
            y: a.pose.y + (b.pose.y - a.pose.y) * f,
            heading: a.pose.heading + wrapAngle(b.pose.heading - a.pose.heading) * f,
          };
        }
        setPose((p) =>
          Math.abs(p.x - next.x) < 1e-4 && Math.abs(p.y - next.y) < 1e-4 && Math.abs(p.heading - next.heading) < 1e-4
            ? p
            : next,
        );
      }
      frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frame);
  }, []);

  return pose;
}
