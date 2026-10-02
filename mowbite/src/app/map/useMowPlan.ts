import type {MowerMap, Point} from '@/hooks/useMowerMap';
import {numParam, type MowerParams} from '@/hooks/useMowerParams';
import type {PastJob} from '@/hooks/useMowHistory';
import {mowerPlan, type PlanRequest} from '@/lib/areaPlan';
import {angleChangedSince} from '@/lib/backups';
import {measuredStripeAngle, stripeAngleDiff} from '@/lib/mowDirection';
import {mowAroundHoles} from '@/lib/mowAround';
import {linkStripes, mowPlan, type MowPlan} from '@/lib/mowPlan';
import {angleInRange, autoMowAngle} from '@/lib/mowStripes';
import {PARAM} from '@/lib/openmower';
import {length} from '@/lib/planProgress';
import {useEffect, useMemo, useRef, useState} from 'react';
import {DEG} from './editing';
import type {AngleMismatch} from './MowSettings';

// The mowing plan shown for the selected area: the mower's own when it offers one (mowing.plan, also for unsaved
// changes), otherwise worked out here like its planner does. Plus the angle it really mows at and whether the last
// real mow went another way.
export function useMowPlan({
  params,
  liveMap,
  shownMap,
  selectedAreaId,
  pastJobs,
  areaProps,
  showStripes,
  previewCorrection,
  draggingPoint,
}: {
  params: MowerParams;
  liveMap: MowerMap | null;
  // the map as shown, with a points reduction preview
  shownMap: MowerMap | null;
  selectedAreaId: string | null;
  pastJobs: PastJob[] | null;
  areaProps: Set<string>;
  showStripes: boolean;
  // extra rotation for the preview when the mower drives differently than calculated
  previewCorrection: number;
  draggingPoint: boolean;
}) {
  const toolWidth = numParam(params, PARAM.toolWidth);
  const angleOffset = numParam(params, PARAM.mowAngleOffset) ?? 0;
  const offsetIsAbsolute = params[PARAM.mowAngleOffsetIsAbsolute] === true;
  const angleIncrement = numParam(params, PARAM.mowAngleIncrement) ?? 0;
  const shownArea = shownMap?.areas.find((a) => a.id === selectedAreaId) ?? null;
  const isMowArea = shownArea?.properties.type === 'mow';
  // a mowing area set to mowable: false gets no plan, it's only driven across
  // while the angle is being changed and a moment after, the stripes show up even when switched off, and for an
  // area that isn't mowed too
  const [angleEditing, setAngleEditing] = useState(false);
  const angleTimer = useRef<ReturnType<typeof setTimeout>>(undefined);
  // moving: right now, for the estimate that follows the slider without waiting for the mower
  const [angleMoving, setAngleMoving] = useState(false);
  const movingTimer = useRef<ReturnType<typeof setTimeout>>(undefined);
  const touchAngle = () => {
    setAngleEditing(true);
    setAngleMoving(true);
    clearTimeout(angleTimer.current);
    clearTimeout(movingTimer.current);
    angleTimer.current = setTimeout(() => setAngleEditing(false), 3000);
    movingTimer.current = setTimeout(() => setAngleMoving(false), 700);
  };
  // an inactive area isn't mowed either
  const mowed = shownArea?.properties.mowable !== false && shownArea?.properties.active !== false;
  const planned = isMowArea && (mowed || angleEditing);
  const stripesOn = showStripes || angleEditing;
  const autoAngle = shownArea ? autoMowAngle(shownArea.outline) : 0;

  // the angle the planner actually gets, see MowingBehavior.cpp
  const plannedAngle = (area: {properties: {angle?: number; angle_min?: number; angle_max?: number}; outline: Point[]}) =>
    angleInRange(
      offsetIsAbsolute ? angleOffset * DEG : (area.properties.angle ?? autoMowAngle(area.outline)) + angleOffset * DEG,
      area.properties.angle_min,
      area.properties.angle_max,
    );

  // check against the last real mow here, a leftover angle increment in checkpoint.bag isn't
  // published anywhere and rotates everything. saved area, not the edited one
  const savedArea = liveMap?.areas.find((a) => a.id === selectedAreaId);
  let mismatch: AngleMismatch | null = null;
  if (savedArea?.properties.type === 'mow' && pastJobs) {
    const passes = savedArea.properties.outline_count ?? numParam(params, PARAM.outlineCount) ?? 0;
    const margin = passes * (toolWidth ?? 0.2) + 0.2;
    // newest mowing stretch first, a job that got interrupted can resume with different settings
    const stretches = pastJobs.flatMap((job) => [...job.segments].reverse().map((seg) => ({job, seg})));
    for (const {job, seg} of stretches) {
      const m = measuredStripeAngle([seg], savedArea.outline, margin);
      if (!m) continue;
      const planned = (((plannedAngle(savedArea) / DEG) % 180) + 180) % 180;
      const diff = stripeAngleDiff(planned, m.angleDeg);
      if (Math.abs(diff) > 8) {
        mismatch = {
          measured: m.angleDeg,
          planned,
          diff,
          date: new Date(job.timestamp * 1000).toLocaleDateString(),
          since: job.timestamp,
        };
      }
      break;
    }
  }
  // not a leftover in checkpoint.bag if the angle got changed on purpose since that mow: a backup from after it
  // (made before a save) has a different one
  const mismatchKey = mismatch && savedArea ? `${savedArea.id} ${mismatch.since} ${savedArea.properties.angle}` : '';
  const [changedOnPurpose, setChangedOnPurpose] = useState<string | null>(null);
  useEffect(() => {
    if (!mismatchKey) return;
    const [id, since, angle] = mismatchKey.split(' ');
    let alive = true;
    void angleChangedSince(id, Number(since), angle === 'undefined' ? undefined : Number(angle)).then(
      (changed) => alive && changed && setChangedOnPurpose(mismatchKey),
    );
    return () => {
      alive = false;
    };
  }, [mismatchKey]);
  if (mismatchKey && changedOnPurpose === mismatchKey) mismatch = null;
  const effectiveAngle = shownArea ? plannedAngle(shownArea) + (mismatch ? previewCorrection : 0) * DEG : 0;

  // the real plan from the mower when it offers one, for the area as it is in the editor right now (saved or not)
  const planRequest = useMemo((): PlanRequest | null => {
    if (!stripesOn || !planned || !shownArea || !shownMap) return null;
    const p = shownArea.properties;
    const req: PlanRequest = {
      outline: shownArea.outline,
      obstacles: [
        ...shownMap.areas
          .filter((a) => a.properties.type === 'obstacle' && a.properties.active !== false && a.outline.length > 2)
          .map((a) => a.outline),
        ...(areaProps.has('mow_around') ? mowAroundHoles(shownArea, shownMap.areas) : []),
      ],
    };
    if (p.angle !== undefined) req.angle = p.angle;
    if (p.outline_count !== undefined) req.outline_count = p.outline_count;
    if (p.outline_overlap_count !== undefined) req.outline_overlap_count = p.outline_overlap_count;
    if (p.outline_offset !== undefined) req.outline_offset = p.outline_offset;
    if (p.angle_min !== undefined && p.angle_max !== undefined) {
      req.angle_min = p.angle_min;
      req.angle_max = p.angle_max;
    }
    return req;
  }, [stripesOn, planned, shownArea, shownMap, areaProps]);
  const planKey = planRequest ? JSON.stringify(planRequest) : '';
  const [fromMower, setFromMower] = useState<{areaId: string | null; plan: MowPlan | null} | null>(null);
  // not while a point is dragged, a new plan redrawn mid-drag makes it stutter
  useEffect(() => {
    if (!planKey || draggingPoint) return;
    const areaId = selectedAreaId;
    let alive = true;
    // while points are typed or clicked only once it settles
    const t = setTimeout(() => {
      void mowerPlan(JSON.parse(planKey)).then(
        (plan) => alive && setFromMower({areaId, plan}),
        () => alive && setFromMower({areaId, plan: null}),
      );
    }, 300);
    return () => {
      alive = false;
      clearTimeout(t);
    };
  }, [planKey, selectedAreaId, draggingPoint]);
  // the last answer for this area stays up while a newer one is on its way, so it doesn't flicker back to the estimate
  // while the angle is moving the estimate follows right away, the mower's plan comes back once it stops
  const realPlan = planKey && !angleMoving && fromMower?.areaId === selectedAreaId ? fromMower.plan : null;
  // the mower's own plan has the angle it really mows at, a leftover increment included, nothing to warn about then
  if (realPlan) mismatch = null;

  // otherwise where the mower will drive, worked out like its planner does (lib/mowPlan)
  const wantPlan = !!(shownMap && shownArea && planned && stripesOn && toolWidth);
  const plan = useMemo((): MowPlan | undefined => {
    if (realPlan) return realPlan;
    if (!wantPlan || !shownMap || !shownArea || !toolWidth) return undefined;
    const holes = [
      ...shownMap.areas
        .filter((a) => a.properties.type === 'obstacle' && a.properties.active !== false && a.outline.length > 2)
        .map((a) => a.outline),
      ...(areaProps.has('mow_around') ? mowAroundHoles(shownArea, shownMap.areas) : []),
    ];
    const p = shownArea.properties;
    const global = (key: string) => numParam(params, PARAM.mowerLogic(key));
    return mowPlan({
      outline: shownArea.outline,
      holes,
      outlineOffset: p.outline_offset ?? global('outline_offset') ?? 0,
      outlineCount: p.outline_count ?? global('outline_count') ?? 0,
      overlapCount: p.outline_overlap_count ?? global('outline_overlap_count') ?? 0,
      toolWidth,
      angle: effectiveAngle,
    });
  }, [realPlan, wantPlan, shownMap, shownArea, toolWidth, params, effectiveAngle, areaProps]);

  // the mower's own plan comes already joined up, the estimate gets its zigzags here
  const stripes = useMemo(
    () =>
      plan && (realPlan ? plan.stripes : toolWidth ? linkStripes(plan.stripes, effectiveAngle, toolWidth) : undefined),
    [plan, realPlan, toolWidth, effectiveAngle],
  );

  // how long the plan is to drive, closed passes included, the drives between the pieces not
  const planLength = useMemo(() => {
    if (!plan || !stripes) return 0;
    const closed = plan.loops.map((o) => (o.length > 1 ? [...o, o[0]] : o));
    return [...closed, ...stripes].reduce((s, o) => s + length(o), 0);
  }, [plan, stripes]);


  return {toolWidth, angleOffset, offsetIsAbsolute, angleIncrement, shownArea, autoAngle, touchAngle, mismatch, realPlan, plan, stripes, planLength};
}
