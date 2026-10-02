import type {Point} from '@/hooks/useMowerMap';
import type {MowPlan} from './mowPlan';
import type {PlanPath} from './planProgress';
import {RPC} from './openmower';
import {callRpc, rpcMethods} from './rpc';

// The real plan for an area, once the mower offers it (RPC.areaPlan). Read like the planner answers
// inside ROS: paths, each an outline pass or a run of stripes, as poses or plain points.
interface PlannerPath {
  is_outline?: boolean | number;
  outline?: boolean;
  points?: ([number, number] | Point)[];
  pose_index?: number[];
  path?: {poses?: {pose: {position: Point}}[]};
}

const points = (p: PlannerPath): Point[] =>
  p.points?.map((q) => (Array.isArray(q) ? {x: q[0], y: q[1]} : {x: q.x, y: q.y})) ??
  p.path?.poses?.map((q) => ({x: q.pose.position.x, y: q.pose.position.y})) ??
  [];

export function readPaths(answer: {paths?: PlannerPath[]} | PlannerPath[]): PlanPath[] {
  const paths = Array.isArray(answer) ? answer : (answer.paths ?? []);
  return paths.map((p) => {
    const pts = points(p);
    // the points come simplified, only pose_index says which pose of the full path each one is
    const index = p.pose_index?.length === pts.length ? p.pose_index : p.path?.poses ? pts.map((_, i) => i) : undefined;
    return {outline: !!(p.is_outline || p.outline), points: pts, index};
  });
}

export function readPlan(answer: {paths?: PlannerPath[]; angle?: number} | PlannerPath[]): MowPlan {
  const loops: Point[][] = [];
  const stripes: [Point, Point][] = [];
  for (const p of readPaths(answer)) {
    if (p.outline) loops.push(p.points);
    else for (let i = 1; i < p.points.length; i++) stripes.push([p.points[i - 1], p.points[i]]);
  }
  return {loops, stripes, angle: Array.isArray(answer) ? undefined : answer.angle};
}

// what mowing.plan takes: a saved area by id, or an area as it is right now in the editor (map.json format,
// missing settings fall back to the mower's global ones)
export type PlanRequest =
  | {area_id: string}
  | {
      outline: Point[];
      obstacles: Point[][];
      angle?: number;
      outline_count?: number;
      outline_overlap_count?: number;
      outline_offset?: number;
      angle_min?: number;
      angle_max?: number;
    };

// null when this mower can't tell, then the editor works it out itself (lib/mowPlan)
export async function mowerPlan(req: PlanRequest): Promise<MowPlan | null> {
  if (!(await rpcMethods())?.has(RPC.areaPlan)) return null;
  return readPlan(await callRpc(RPC.areaPlan, req, 20000));
}

// the plan in the order it's driven, for showing the progress of a run
export async function mowerPaths(areaId: string): Promise<PlanPath[] | null> {
  if (!(await rpcMethods())?.has(RPC.areaPlan)) return null;
  return readPaths(await callRpc(RPC.areaPlan, {area_id: areaId}, 20000));
}
