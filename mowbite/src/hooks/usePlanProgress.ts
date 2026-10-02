'use client';

import {mowerPaths} from '@/lib/areaPlan';
import type {MowerEvent} from '@/lib/events';
import {bladeSeconds, saveRate, savedRate, splitPlan, type PlanPath, type PlanProgress} from '@/lib/planProgress';
import {useEffect, useMemo, useState} from 'react';
import type {MowerMap} from './useMowerMap';
import type {MowerState} from './useMowerState';

// how far the mower got with the area it's mowing, when it can hand out its plan (mowing.plan)
export function usePlanProgress(
  state: MowerState | null,
  map: MowerMap | null,
  events: MowerEvent[] | undefined,
): (PlanProgress & {secondsLeft: number | null}) | null {
  // current_area counts the mowing areas in map order, inactive ones included
  const areaId =
    state?.current_state === 'MOWING' && map && (state.current_area ?? -1) >= 0
      ? map.areas.filter((a) => a.properties.type === 'mow')[state.current_area!]?.id
      : undefined;
  const [plan, setPlan] = useState<{areaId: string; paths: PlanPath[]} | null>(null);

  useEffect(() => {
    if (!areaId) return;
    let gone = false;
    mowerPaths(areaId)
      .then((paths) => !gone && setPlan(paths ? {areaId, paths} : null))
      .catch(() => !gone && setPlan(null));
    return () => {
      gone = true;
    };
    // again when the map changes, the plan might have too
  }, [areaId, map]);

  const path = state?.current_path ?? -1;
  const pose = state?.current_path_index ?? 0;
  const progress = useMemo(
    () => (areaId && plan?.areaId === areaId && path >= 0 ? splitPlan(plan.paths, path, pose) : null),
    [areaId, plan, path, pose],
  );

  // time left from how fast the plan got done while the blades ran in this area (event history), the same on every
  // device. checked every 10 s. a rate from a good while of mowing is kept for the editor
  const [now, setNow] = useState(() => Date.now() / 1000);
  useEffect(() => {
    const every = setInterval(() => setNow(Date.now() / 1000), 10000);
    return () => clearInterval(every);
  }, []);
  const seconds = areaId && events ? bladeSeconds(events, areaId, now) : 0;
  const rate = progress && seconds > 60 && progress.doneLength > 5 ? progress.doneLength / seconds : null;
  useEffect(() => {
    if (rate && seconds > 600) saveRate(rate);
  }, [rate, seconds]);

  return useMemo(() => {
    if (!progress) return null;
    const r = rate ?? savedRate();
    return {...progress, secondsLeft: r ? progress.todoLength / r : null};
  }, [progress, rate]);
}
