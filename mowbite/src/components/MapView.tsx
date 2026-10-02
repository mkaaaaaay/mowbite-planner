'use client';

import type {MowerMap, Point} from '@/hooks/useMowerMap';
import {useComputedSpeed} from '@/hooks/useComputedSpeed';
import {useEasedPose} from '@/hooks/useEasedPose';
import {containsPoint, polygonArea} from '@/lib/geometry';
import {settingsStore} from '@/lib/settings';
import {dockIcon, mowerIcon} from './mapIcons';
import {availableSources, imageryTiles, type Datum} from '@/lib/imagery';
import {handleRadius, meterGrid} from '@/lib/mapGrid';
import MapControls, {layerOn, type Layer} from './MapControls';
import {useMapPrefs} from './useMapPrefs';
import {useCallback, useEffect, useMemo, useRef, useState, useSyncExternalStore} from 'react';
import styles from './MapView.module.css';
import {tr, useLang} from '@/lib/i18n';
import {tick} from '@/lib/haptics';

interface MapViewProps {
  map: MowerMap;
  mower?: {x: number; y: number; heading: number};
  // the emergency stop is active, some mower icons show it
  emergency?: boolean;
  selectedAreaId?: string | null;
  onSelectArea?: (id: string) => void;
  onMoveVertex?: (areaId: string, vertexIndex: number, x: number, y: number) => void;
  onDragStart?: () => void;
  // true while a point of an area is being dragged
  onDragging?: (on: boolean) => void;
  // a point drag that turned into a pinch: undo what the drag changed
  onDragCancel?: () => void;
  onInsertVertex?: (areaId: string, vertexIndex: number, x: number, y: number) => void;
  onDeleteVertex?: (areaId: string, vertexIndex: number) => void;
  // clicks report map coords instead of selecting (split line, new area)
  pickingPoints?: boolean;
  pendingPoints?: Point[];
  onCanvasClick?: (x: number, y: number) => void;
  onMovePending?: (index: number, x: number, y: number) => void;
  onInsertPending?: (index: number, x: number, y: number) => void;
  // fixed window around the mower instead of fitting the whole map, a button lets the map move freely
  follow?: boolean;
  followSpanMeters?: number;
  // oldest first
  // the live trail in pieces (see useMowerTrack), finished pieces keep their identity
  track?: readonly (Point & {b?: boolean})[][];
  // a recorded job instead of the live trail, mowed parts solid, driving without blades dashed
  pastTrack?: {points: Point[]; blades: boolean}[];
  // wheel / pinch zoom, drag to pan
  zoomable?: boolean;
  // mowing direction preview
  stripes?: Point[][];
  // the outline passes of the mowing plan, drawn with the stripes
  loops?: Point[][];
  // how far the current run got in the mower's plan: what's left is drawn, the planned part it has done only if
  // switched on in the layer menu (the track shows what it really drove)
  progress?: {done: Point[][]; todo: Point[][]};
  // shapes an edit would give (split pieces, merge result), drawn in two alternating colors
  preview?: Point[][];
  // click on the map where there's no area
  onClickEmpty?: () => void;
  // gps datum of the map, enables the aerial imagery option
  datum?: Datum;
  // numbers drawn in areas, e.g. mowing order
  orderLabels?: Record<string, number>;
  // spots to point out, e.g. where an error happened (red, pulsing)
  markers?: Point[];
  // start zoomed in around this point instead of showing the whole map
  focus?: Point;
  // zoom and position are kept under this key while the app runs, e.g. across a visit to the settings
  viewKey?: string;
  // what the mower draws itself, e.g. the lines of an area recording
  overlay?: {points: Point[]; color: string; closed: boolean}[];
}

interface Bounds {
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
}

// square viewBox window, in svg units
interface View {
  x: number;
  y: number;
  size: number;
}

const MAX_ZOOM = 40;
// center and width in meters, so a changed map doesn't move what was looked at
const savedViews = new Map<string, {x: number; y: number; span: number}>();
// dragging.areaId while a point of the line being drawn is dragged
const PENDING = '__pending';

type Run = {points: string; blades: boolean};
// drawn runs of each piece of the live trail per map geometry, pieces that are done are never worked out again
const runCache = new WeakMap<object, Map<string, Run[]>>();

const AREA_CLASS: Record<string, string> = {
  mow: styles.mowArea,
  nav: styles.navArea,
  obstacle: styles.obstacleArea,
  draft: styles.draftArea,
};
const OVERLAY_CLASS: Record<string, string> = {
  green: styles.overlayOutline,
  red: styles.overlayObstacle,
  blue: styles.overlayLive,
};
const LOUPE_PX = 120;
const LOUPE_ZOOM = 2.5;

const PADDING = 20;
const MOWER_SIZE = 0.4; // m
const WIDTH = 400;
const HEIGHT = 400;


type Fit = {scale: number; padX: number; padY: number; minX: number; minY: number};
type Meters = {x: number; y: number; span: number};

// a view (center and width in meters) in svg units and back, for the map fitted as given
function viewFromMeters(m: Meters, f: Fit): View {
  const size = m.span * f.scale;
  return {x: (m.x - f.minX) * f.scale + f.padX - size / 2, y: HEIGHT - ((m.y - f.minY) * f.scale + f.padY) - size / 2, size};
}

function viewToMeters(v: View, f: Fit): Meters {
  return {x: (v.x + v.size / 2 - f.padX) / f.scale + f.minX, y: (HEIGHT - v.y - v.size / 2 - f.padY) / f.scale + f.minY, span: v.size / f.scale};
}

export default function MapView({
  map,
  mower,
  emergency,
  selectedAreaId,
  onSelectArea,
  onMoveVertex,
  onDragStart,
  onDragging,
  onDragCancel,
  onInsertVertex,
  onDeleteVertex,
  pickingPoints,
  pendingPoints,
  onCanvasClick,
  onMovePending,
  onInsertPending,
  follow = false,
  followSpanMeters = 6,
  track,
  pastTrack,
  progress,
  zoomable = false,
  stripes,
  loops,
  preview,
  overlay,
  markers,
  focus,
  viewKey,
  onClickEmpty,
  datum,
  orderLabels,
}: MapViewProps) {
  useLang();
  const svgRef = useRef<SVGSVGElement | null>(null);
  // the svg isn't there on the first render while the map is still loading, so effects that need it
  // depend on this instead of running once on mount
  const [svgEl, setSvgEl] = useState<SVGSVGElement | null>(null);
  const attachSvg = useCallback((el: SVGSVGElement | null) => {
    svgRef.current = el;
    setSvgEl(el);
  }, []);
  // bounds frozen while dragging so the map doesn't rescale under the cursor
  const [dragging, setDragging] = useState<{areaId: string; index: number; bounds: Bounds} | null>(null);
  const drag = useRef<{
    pointerId: number;
    startX: number;
    startY: number;
    lastX: number;
    lastY: number;
    moved: boolean;
    touch: boolean;
    // an edge midpoint was grabbed, the new point only gets added once the finger moves (or on a tap)
    insert: {x: number; y: number} | null;
    // where the point was before, for putting a drawn point back
    orig: {x: number; y: number} | null;
  }>({pointerId: -1, startX: 0, startY: 0, lastX: 0, lastY: 0, moved: false, touch: false, insert: null, orig: null});
  // finger position while dragging on touch, drives the loupe
  const [finger, setFinger] = useState<{x: number; y: number; width: number} | null>(null);
  // tapped point, gets a delete button
  const [active, setActive] = useState<{areaId: string; index: number} | null>(null);
  // the user's zoom and pan, center and width in meters: a map that changes size (a point dragged past the edge,
  // a new area drawn outside) rescales the drawing, but what's looked at stays put
  const [viewMeters, setViewMeters] = useState<Meters | null>(null);
  const settings = useSyncExternalStore(settingsStore.subscribe, settingsStore.snapshot, settingsStore.serverSnapshot);
  // the view from the last visit, until zoomed, panned or reset. a spot to show wins over it
  const [restore, setRestore] = useState(() => (viewKey && !focus ? (savedViews.get(viewKey) ?? null) : null));
  // in follow mode zooming changes how many meters around the mower are shown
  const [followZoom, setFollowZoom] = useState(1);
  // follow mode switched off by hand, back on with the next drive
  const [free, setFree] = useState(false);
  if (free && !follow) setFree(false);
  const following = follow && !free;
  const {imagery, setImagery, showGrid, toggleGrid, hidden, toggleLayer, planStyle, setPlanStyle} = useMapPrefs();
  const [svgPx, setSvgPx] = useState(0);
  const [layersOpen, setLayersOpen] = useState(false);
  const pointers = useRef(new Map<number, {x: number; y: number}>());
  const gesture = useRef<{moved: boolean; pinchDist: number | null}>({moved: false, pinchDist: null});
  const smoothedMower = useEasedPose(mower ?? {x: 0, y: 0, heading: 0});
  const displayMower = mower ? smoothedMower : undefined;
  const mowerSpeed = useComputedSpeed(mower);
  // which way a side view faces. it only turns once the mower clearly heads the other way, so it doesn't flicker while
  // the mower drives up or down the map
  const [facing, setFacing] = useState(1);
  const across = Math.cos(displayMower?.heading ?? 0);
  if (across > 0.3 && facing !== 1) setFacing(1);
  if (across < -0.3 && facing !== -1) setFacing(-1);

  // fitted to the map only, so the geometry doesn't change while the mower moves. follow mode
  // moves the view instead
  const fit = useMemo(() => {
    const points = [...map.areas.flatMap((a) => a.outline), ...(pendingPoints ?? [])];
    if (!points.length) return null;
    const xs = points.map((p) => p.x);
    const ys = points.map((p) => p.y);
    return {minX: Math.min(...xs), maxX: Math.max(...xs), minY: Math.min(...ys), maxY: Math.max(...ys)};
  }, [map, pendingPoints]);

  const center = displayMower ?? {x: 0, y: 0};
  const {minX, maxX, minY, maxY}: Bounds = dragging?.bounds ??
    fit ?? {
      minX: center.x - followSpanMeters / 2,
      maxX: center.x + followSpanMeters / 2,
      minY: center.y - followSpanMeters / 2,
      maxY: center.y + followSpanMeters / 2,
    };

  const scaleX = (WIDTH - 2 * PADDING) / (maxX - minX || 1);
  const scaleY = (HEIGHT - 2 * PADDING) / (maxY - minY || 1);
  const scale = Math.min(scaleX, scaleY);
  // center whatever doesn't fill the square
  const padX = PADDING + (WIDTH - 2 * PADDING - (maxX - minX) * scale) / 2;
  const padY = PADDING + (HEIGHT - 2 * PADDING - (maxY - minY) * scale) / 2;

  // the view between meters and svg units. setView may run from the wheel handler set up earlier, so it takes
  // the fit from a ref the last render left
  const drawn: Fit = {scale, padX, padY, minX, minY};
  const fitRef = useRef(drawn);
  useEffect(() => {
    fitRef.current = {scale, padX, padY, minX, minY};
  }, [scale, padX, padY, minX, minY]);
  const view = viewMeters ? viewFromMeters(viewMeters, drawn) : null;
  const setView = (next: View | null | ((prev: View | null) => View)) =>
    setViewMeters((prev) => {
      const f = fitRef.current;
      const v = typeof next === 'function' ? next(prev ? viewFromMeters(prev, f) : null) : next;
      return v ? viewToMeters(v, f) : null;
    });

  // map y points north, svg y points down
  const toScreen = (x: number, y: number): [number, number] => [(x - minX) * scale + padX, HEIGHT - ((y - minY) * scale + padY)];
  const toLocal = (sx: number, sy: number): [number, number] => [(sx - padX) / scale + minX, (HEIGHT - sy - padY) / scale + minY];

  // about 12 m around the focus point, until the user zooms or pans themselves
  const home: View | null = focus
    ? (() => {
        const [fx, fy] = toScreen(focus.x, focus.y);
        const size = Math.min(WIDTH, 12 * scale);
        return {x: fx - size / 2, y: fy - size / 2, size};
      })()
    : null;

  const restored: View | null = restore
    ? (() => {
        const [cx, cy] = toScreen(restore.x, restore.y);
        const size = restore.span * scale;
        return {x: cx - size / 2, y: cy - size / 2, size};
      })()
    : null;

  // what's shown: the user's zoom, or in follow mode a window around the mower
  const base = view ?? restored ?? home;
  let shown = base;
  if (following && displayMower) {
    const [sx, sy] = toScreen(displayMower.x, displayMower.y);
    const size = followSpanMeters * followZoom * scale;
    shown = {x: sx - size / 2, y: sy - size / 2, size};
  }

  // outlines as svg point strings, rebuilt only when the map or the fit changes, not every frame
  const outlinePoints = useMemo(() => {
    const out = new Map<string, string>();
    for (const a of map.areas) {
      out.set(a.id, a.outline.map((p) => `${(p.x - minX) * scale + padX},${HEIGHT - ((p.y - minY) * scale + padY)}`).join(' '));
    }
    return out;
  }, [map, minX, minY, scale, padX, padY]);
  // the live trail cut where the blades go on or off, driving without them is drawn dashed. worked out per
  // piece and kept for pieces that didn't change, so a long trail costs the same as a short one while driving
  const trackRuns = useMemo(() => {
    const runs: Run[] = [];
    if (!track) return runs;
    const key = `${minX},${minY},${scale},${padX},${padY}`;
    const at = (p: Point) => `${(p.x - minX) * scale + padX},${HEIGHT - ((p.y - minY) * scale + padY)}`;
    for (const piece of track) {
      let byView = runCache.get(piece);
      if (!byView) runCache.set(piece, (byView = new Map()));
      let cached = byView.get(key);
      if (!cached) {
        const own: Run[] = [];
        let start = 0;
        for (let i = 1; i <= piece.length; i++) {
          if (i < piece.length && (piece[i].b ?? true) === (piece[start].b ?? true)) continue;
          // include the next point so the runs connect
          const pts = piece.slice(start, Math.min(i + 1, piece.length));
          if (pts.length >= 2) own.push({points: pts.map(at).join(' '), blades: piece[start].b ?? true});
          start = i;
        }
        cached = own;
        // one entry per map size it's drawn at, the dashboard and the map page differ
        if (byView.size > 3) byView.clear();
        byView.set(key, cached);
      }
      runs.push(...cached);
    }
    return runs;
  }, [track, minX, minY, scale, padX, padY]);

  const selectedArea = map.areas.find((a) => a.id === selectedAreaId);
  const activeIndex =
    active && selectedArea && active.areaId === selectedArea.id && active.index < selectedArea.outline.length
      ? active.index
      : null;

  // keeps handles the same size at any zoom
  const k = shown ? shown.size / WIDTH : 1;
  const pxPerMeter = scale / k;

  const clientToSvg = (clientX: number, clientY: number): [number, number] | null => {
    const svg = svgRef.current;
    const ctm = svg?.getScreenCTM();
    if (!svg || !ctm) return null;
    const pt = svg.createSVGPoint();
    pt.x = clientX;
    pt.y = clientY;
    const local = pt.matrixTransform(ctm.inverse());
    return [local.x, local.y];
  };

  const clientToLocal = (clientX: number, clientY: number): [number, number] | null => {
    const p = clientToSvg(clientX, clientY);
    return p ? toLocal(p[0], p[1]) : null;
  };

  // big areas first so small ones on top stay clickable
  const areasBySize = useMemo(
    () => [...map.areas].sort((a, b) => polygonArea(b.outline) - polygonArea(a.outline)),
    [map],
  );

  // clicking the selected area again goes to the next one underneath
  const selectAt = (clickedId: string, clientX: number, clientY: number) => {
    const local = clientToLocal(clientX, clientY);
    if (clickedId !== selectedAreaId || !local) {
      onSelectArea?.(clickedId);
      return;
    }
    const stack = areasBySize.filter((a) => containsPoint(a.outline, local[0], local[1])).reverse();
    const i = stack.findIndex((a) => a.id === selectedAreaId);
    if (stack.length > 1) onSelectArea?.(stack[(i + 1) % stack.length].id);
  };

  // midpoints are only offered where there's room for them
  const showMidpoint = (a: Point, b: Point) => Math.hypot(b.x - a.x, b.y - a.y) * pxPerMeter >= 30;

  // nearest vertex or edge midpoint within finger/mouse reach, handles themselves are too small to hit
  const pickHandle = (clientX: number, clientY: number, touch: boolean) => {
    const svg = svgRef.current;
    const local = clientToLocal(clientX, clientY);
    if (!svg || !local || !selectedArea || !onMoveVertex || pickingPoints) return null;

    const reach = (touch ? 22 : 12) / ((pxPerMeter * svg.getBoundingClientRect().width) / WIDTH);
    const o = selectedArea.outline;
    let best: {mid: boolean; index: number; x: number; y: number} | null = null;
    let bestDist = reach;

    o.forEach((p, i) => {
      const d = Math.hypot(p.x - local[0], p.y - local[1]);
      if (d < bestDist) {
        bestDist = d;
        best = {mid: false, index: i, x: p.x, y: p.y};
      }
      const next = o[(i + 1) % o.length];
      if (!onInsertVertex || !showMidpoint(p, next)) return;
      const mx = (p.x + next.x) / 2;
      const my = (p.y + next.y) / 2;
      const dm = Math.hypot(mx - local[0], my - local[1]);
      if (dm < bestDist) {
        bestDist = dm;
        best = {mid: true, index: i, x: mx, y: my};
      }
    });
    return best as {mid: boolean; index: number; x: number; y: number} | null;
  };

  // same for the points of a line that's being drawn (split, new area)
  const pickPending = (clientX: number, clientY: number, touch: boolean) => {
    const svg = svgRef.current;
    const local = clientToLocal(clientX, clientY);
    if (!svg || !local || !pickingPoints || !pendingPoints?.length || !onMovePending) return null;

    const reach = (touch ? 22 : 12) / ((pxPerMeter * svg.getBoundingClientRect().width) / WIDTH);
    let best: {mid: boolean; index: number; x: number; y: number} | null = null;
    let bestDist = reach;
    pendingPoints.forEach((p, i) => {
      const d = Math.hypot(p.x - local[0], p.y - local[1]);
      if (d < bestDist) {
        bestDist = d;
        best = {mid: false, index: i, x: p.x, y: p.y};
      }
      const next = pendingPoints[i + 1];
      if (!next || !onInsertPending || !showMidpoint(p, next)) return;
      const mx = (p.x + next.x) / 2;
      const my = (p.y + next.y) / 2;
      const dm = Math.hypot(mx - local[0], my - local[1]);
      if (dm < bestDist) {
        bestDist = dm;
        best = {mid: true, index: i, x: mx, y: my};
      }
    });
    return best as {mid: boolean; index: number; x: number; y: number} | null;
  };

  // zoom by factor (<1 = in) keeping the svg point (px, py) where it is on screen
  const zoomAt = (factor: number, px: number, py: number) => {
    if (following) {
      setFollowZoom((z) => Math.min(8, Math.max(0.25, z * factor)));
      return;
    }
    setView((prev) => {
      const v = prev ?? base ?? {x: 0, y: 0, size: WIDTH};
      const size = Math.min(WIDTH * 4, Math.max(WIDTH / MAX_ZOOM, v.size * factor));
      const f = size / v.size;
      return {x: px - (px - v.x) * f, y: py - (py - v.y) * f, size};
    });
  };

  const zoomCenter = (factor: number) => {
    const v = base ?? {x: 0, y: 0, size: WIDTH};
    zoomAt(factor, v.x + v.size / 2, v.y + v.size / 2);
  };

  useEffect(() => {
    if (!svgEl) return;
    const ro = new ResizeObserver(() => setSvgPx(svgEl.getBoundingClientRect().width));
    ro.observe(svgEl);
    return () => ro.disconnect();
  }, [svgEl]);

  useEffect(() => {
    const svg = svgEl;
    if (!zoomable || !svg) return;

    // react's onWheel is passive, can't preventDefault the page scroll there
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const p = clientToSvg(e.clientX, e.clientY);
      if (p) zoomAt(Math.exp(e.deltaY * 0.0015), p[0], p[1]);
    };
    svg.addEventListener('wheel', onWheel, {passive: false});
    return () => svg.removeEventListener('wheel', onWheel);
    // eslint-disable-next-line react-hooks/exhaustive-deps -- zoomAt only uses setters and following
  }, [zoomable, following, svgEl]);

  useEffect(() => {
    if (viewKey && viewMeters) savedViews.set(viewKey, viewMeters);
  }, [viewKey, viewMeters]);

  const startDrag = (e: React.PointerEvent<SVGSVGElement>, areaId: string, hit: {mid: boolean; index: number; x: number; y: number}) => {
    const at = {x: hit.x, y: hit.y};
    drag.current = {
      pointerId: e.pointerId,
      startX: e.clientX,
      startY: e.clientY,
      lastX: e.clientX,
      lastY: e.clientY,
      moved: false,
      touch: e.pointerType !== 'mouse',
      insert: hit.mid ? at : null,
      orig: hit.mid ? null : at,
    };
    gesture.current.moved = true; // eat the click that follows
    if (e.pointerType === 'touch') tick();
    setDragging({areaId, index: hit.mid ? hit.index + 1 : hit.index, bounds: {minX, maxX, minY, maxY}});
  };

  // a second finger while a point is held means the first one was the start of a pinch
  const cancelDrag = (e: React.PointerEvent<SVGSVGElement>) => {
    if (!dragging) return;
    const d = drag.current;
    if (d.moved) {
      if (dragging.areaId !== PENDING) onDragCancel?.();
      else if (d.orig) onMovePending?.(dragging.index, d.orig.x, d.orig.y);
    }
    setDragging(null);
    setFinger(null);
    setActive(null);
    const first = d.pointerId;
    d.pointerId = -1;
    if (!zoomable) return;
    pointers.current.clear();
    pointers.current.set(first, {x: d.lastX, y: d.lastY});
    pointers.current.set(e.pointerId, {x: e.clientX, y: e.clientY});
    gesture.current = {moved: true, pinchDist: null};
  };

  const onPointerDown = (e: React.PointerEvent<SVGSVGElement>) => {
    const touch = e.pointerType !== 'mouse';
    if (dragging) {
      if (touch && drag.current.touch && e.pointerId !== drag.current.pointerId) cancelDrag(e);
      return;
    }

    const pendingHit = pointers.current.size === 0 ? pickPending(e.clientX, e.clientY, touch) : null;
    if (pendingHit) {
      startDrag(e, PENDING, pendingHit);
      return;
    }

    const hit = selectedArea && pointers.current.size === 0 ? pickHandle(e.clientX, e.clientY, touch) : null;
    if (hit && selectedArea) {
      startDrag(e, selectedArea.id, hit);
      return;
    }

    if (!zoomable) return;
    pointers.current.set(e.pointerId, {x: e.clientX, y: e.clientY});
    if (pointers.current.size === 1) gesture.current = {moved: false, pinchDist: null};
  };

  const onPointerMove = (e: React.PointerEvent<SVGSVGElement>) => {
    const prev = pointers.current.get(e.pointerId);
    if (!zoomable || !prev || dragging) return;
    const svg = svgRef.current;
    if (!svg) return;

    pointers.current.set(e.pointerId, {x: e.clientX, y: e.clientY});
    const pts = [...pointers.current.values()];

    if (pts.length >= 2) {
      const dist = Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y);
      const mid = clientToSvg((pts[0].x + pts[1].x) / 2, (pts[0].y + pts[1].y) / 2);
      if (gesture.current.pinchDist && mid) zoomAt(gesture.current.pinchDist / dist, mid[0], mid[1]);
      gesture.current = {moved: true, pinchDist: dist};
      return;
    }

    const dx = e.clientX - prev.x;
    const dy = e.clientY - prev.y;
    if (!gesture.current.moved && Math.hypot(dx, dy) < 4) {
      // not a pan yet, don't eat the click
      pointers.current.set(e.pointerId, prev);
      return;
    }
    gesture.current.moved = true;
    if (following) return; // the view is pinned to the mower
    const v = base ?? {x: 0, y: 0, size: WIDTH};
    const unitsPerPx = v.size / svg.getBoundingClientRect().width;
    setView({...v, x: v.x - dx * unitsPerPx, y: v.y - dy * unitsPerPx});
  };

  const onPointerEnd = (e: React.PointerEvent<SVGSVGElement>) => {
    pointers.current.delete(e.pointerId);
    if (pointers.current.size < 2) gesture.current.pinchDist = null;
  };

  useEffect(() => {
    if (!dragging) return;

    const addPoint = (x: number, y: number) => {
      if (dragging.areaId === PENDING) onInsertPending?.(dragging.index, x, y);
      else onInsertVertex?.(dragging.areaId, dragging.index, x, y);
    };

    const onMove = (e: PointerEvent) => {
      const d = drag.current;
      if (e.pointerId !== d.pointerId) return;
      d.lastX = e.clientX;
      d.lastY = e.clientY;
      if (!d.moved) {
        // a finger needs a bit more way, so the second finger of a pinch comes in before anything changes
        if (Math.hypot(e.clientX - d.startX, e.clientY - d.startY) < (d.touch ? 10 : 4)) return;
        d.moved = true;
        // adding a point takes its own undo snapshot, drawn points aren't in the map history at all
        if (d.insert) addPoint(d.insert.x, d.insert.y);
        else if (dragging.areaId !== PENDING) onDragStart?.();
        if (dragging.areaId !== PENDING) onDragging?.(true);
      }
      const local = clientToLocal(e.clientX, e.clientY);
      if (local && dragging.areaId === PENDING) onMovePending?.(dragging.index, local[0], local[1]);
      else if (local) onMoveVertex?.(dragging.areaId, dragging.index, local[0], local[1]);
      const rect = svgRef.current?.getBoundingClientRect();
      if (d.touch && rect) setFinger({x: e.clientX - rect.left, y: e.clientY - rect.top, width: rect.width});
    };
    const onUp = (e: PointerEvent) => {
      const d = drag.current;
      if (e.pointerId !== d.pointerId) return;
      // handled, a repeated up for the same finger does nothing
      d.pointerId = -1;
      if (!d.moved && d.insert) {
        // a tap on an edge midpoint adds the point there
        addPoint(d.insert.x, d.insert.y);
        setActive(null);
      } else if (!d.moved && dragging.areaId !== PENDING) {
        const same = active?.areaId === dragging.areaId && active.index === dragging.index;
        setActive(same ? null : {areaId: dragging.areaId, index: dragging.index});
      } else {
        setActive(null);
      }
      setDragging(null);
      setFinger(null);
      onDragging?.(false);
    };
    const onCancel = (e: PointerEvent) => {
      if (e.pointerId !== drag.current.pointerId) return;
      drag.current.pointerId = -1;
      setDragging(null);
      setFinger(null);
      onDragging?.(false);
    };

    window.addEventListener('pointermove', onMove);
    window.addEventListener('pointerup', onUp);
    window.addEventListener('pointercancel', onCancel);
    return () => {
      window.removeEventListener('pointermove', onMove);
      window.removeEventListener('pointerup', onUp);
      window.removeEventListener('pointercancel', onCancel);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [dragging, scale, minX, minY]);

  if (map.areas.length === 0 && !displayMower && !pendingPoints?.length) return null;


  const sources = datum ? availableSources(datum, settings.imagery) : [];
  const source = imagery && sources.length ? (sources.includes(imagery) ? imagery : sources[0]) : null;
  let tiles: ReturnType<typeof imageryTiles> = [];
  if (zoomable && datum && source && svgPx > 0) {
    const v = shown ?? {x: 0, y: 0, size: WIDTH};
    const [left, bottom] = toLocal(v.x, v.y + v.size);
    const [right, top] = toLocal(v.x + v.size, v.y);
    const metersPerPixel = 1 / ((pxPerMeter * svgPx) / WIDTH);
    tiles = imageryTiles(source, datum, {left, right, bottom, top, metersPerPixel}, settings.imagery);
  }

  let grid: ReturnType<typeof meterGrid> | null = null;
  if (zoomable && showGrid) {
    const v = shown ?? {x: 0, y: 0, size: WIDTH};
    const [left, bottom] = toLocal(v.x, v.y + v.size);
    const [right, top] = toLocal(v.x + v.size, v.y);
    grid = meterGrid({left, right, bottom, top}, pxPerMeter);
  }

  const vertexR = selectedArea ? handleRadius(selectedArea.outline, pxPerMeter) : 2.5;

  // k = svg units per screen unit, the loupe passes its own so handles don't get magnified
  const renderContent = (k: number) => (
    <>
      {tiles.length > 0 && (
        <g className={styles.imagery}>
          {tiles.map((t) => {
            const [p0, p1, p2] = t.corners.map((c) => toScreen(c.x, c.y));
            const m = [p1[0] - p0[0], p1[1] - p0[1], p2[0] - p0[0], p2[1] - p0[1], p0[0], p0[1]].join(' ');
            return (
              <image key={t.href} href={t.href} width={1} height={1} preserveAspectRatio="none" transform={`matrix(${m})`} />
            );
          })}
        </g>
      )}
        {areasBySize.filter((area) => !hidden.has(area.properties.type as Layer)).map((area) => (
          <polygon
            key={area.id}
            points={outlinePoints.get(area.id)}
            className={[
              AREA_CLASS[area.properties.type ?? 'draft'] ?? styles.draftArea,
              area.properties.active === false ? styles.inactive : '',
              area.properties.mowable === false ? styles.skipMowing : '',
            ]
              .filter(Boolean)
              .join(' ')}
            data-area
            onClick={(e) => {
              if (!pickingPoints) selectAt(area.id, e.clientX, e.clientY);
            }}
          />
        ))}

        {loops && loops.length > 0 && !hidden.has('stripes') && (
          <path
            className={styles.stripes}
            d={loops
              .map((o) => o.map((p, i) => `${i ? 'L' : 'M'}${toScreen(p.x, p.y).join(' ')}`).join('') + 'Z')
              .join('')}
          />
        )}
        {stripes && stripes.length > 0 && !hidden.has('stripes') && (
          <path
            className={styles.stripes}
            d={stripes.map((o) => o.map((p, i) => `${i ? 'L' : 'M'}${toScreen(p.x, p.y).join(' ')}`).join('')).join('')}
          />
        )}

        {selectedArea && (
          <polygon
            points={selectedArea.outline.map((p) => toScreen(p.x, p.y).join(',')).join(' ')}
            className={styles.selected}
          />
        )}

        {orderLabels &&
          !hidden.has('order') &&
          map.areas.map((area) => {
            const n = orderLabels[area.id];
            if (!n || !area.outline.length) return null;
            const cx = area.outline.reduce((s, p) => s + p.x, 0) / area.outline.length;
            const cy = area.outline.reduce((s, p) => s + p.y, 0) / area.outline.length;
            const [sx, sy] = toScreen(cx, cy);
            return (
              <g key={'n' + area.id} className={styles.orderLabel}>
                <circle cx={sx} cy={sy} r={8 * k} />
                <text x={sx} y={sy} fontSize={10 * k} dy="0.35em">
                  {n}
                </text>
              </g>
            );
          })}

        {grid && (
          <g className={styles.grid}>
            {grid.xs.map((x) => {
              const sx = toScreen(x, 0)[0];
              return <line key={'x' + x} x1={sx} x2={sx} y1={-10 * HEIGHT} y2={11 * HEIGHT} />;
            })}
            {grid.ys.map((y) => {
              const sy = toScreen(0, y)[1];
              return <line key={'y' + y} x1={-10 * WIDTH} x2={11 * WIDTH} y1={sy} y2={sy} />;
            })}
          </g>
        )}

        {map.docking_stations.map((station) => {
          const [sx, sy] = toScreen(station.position.x, station.position.y);
          return (
            <g key={station.id} className={styles.dock} transform={`translate(${sx} ${sy}) scale(${8 * k * (settings.icons?.dockSize ?? 1)})`}>
              {dockIcon(settings.icons?.dock).draw()}
            </g>
          );
        })}

        {progress && !hidden.has('stripes') && (
          <>
            <path
              className={[styles.planTodo, styles[planStyle]].join(' ')}
              d={progress.todo.map((o) => o.map((p, i) => `${i ? 'L' : 'M'}${toScreen(p.x, p.y).join(' ')}`).join('')).join('')}
            />
            {layerOn(hidden, 'planDone') && (
              <path
                className={styles.planDone}
                d={progress.done.map((o) => o.map((p, i) => `${i ? 'L' : 'M'}${toScreen(p.x, p.y).join(' ')}`).join('')).join('')}
              />
            )}
          </>
        )}

        {!hidden.has('track') &&
          trackRuns
            .filter((run) => run.blades || !hidden.has('transit'))
            .map((run, i) => (
              <polyline key={'run' + i} points={run.points} className={run.blades ? styles.track : styles.transit} />
            ))}
        {!hidden.has('track') &&
          pastTrack?.map((seg, i) =>
          seg.points.length >= 2 && (seg.blades || !hidden.has('transit')) ? (
            <polyline
              key={'past' + i}
              points={seg.points.map((p) => toScreen(p.x, p.y).join(',')).join(' ')}
              className={seg.blades ? styles.track : styles.transit}
            />
          ) : null,
        )}

        {displayMower &&
          (() => {
            const [sx, sy] = toScreen(displayMower.x, displayMower.y);
            // same size on screen like the dock, or its real size (never smaller than a few pixels)
            const base = settings.icons?.mowerRealSize ? Math.max(MOWER_SIZE * scale, 7 * k) : 8 * k;
            const size = base * (settings.icons?.mowerSize ?? 1);
            // svg y points down, so the map's ccw heading becomes a cw rotation
            const deg = (-displayMower.heading * 180) / Math.PI;
            const icon = mowerIcon(settings.icons?.mower);
            // a side view heading left would stand on its head, mirrored it keeps its feet on the ground. a figure
            // isn't turned with the heading at all, it only looks left or right. css, so it turns around instead of
            // flipping over at once
            const turn = !icon.side ? undefined : icon.upright ? `scaleX(${facing})` : `scaleY(${facing})`;
            return (
              <g className={styles.mower} transform={`translate(${sx} ${sy}) rotate(${icon.upright ? 0 : deg}) scale(${size})`}>
                <g className={styles.turn} style={turn ? {transform: turn} : undefined}>
                  {icon.draw({speed: mowerSpeed, emergency, blades: track?.at(-1)?.at(-1)?.b === true})}
                </g>
              </g>
            );
          })()}

        {selectedArea && !pickingPoints && (
          <>
            {onInsertVertex &&
              selectedArea.outline.map((p, i) => {
                const next = selectedArea.outline[(i + 1) % selectedArea.outline.length];
                if (!showMidpoint(p, next)) return null;
                const [sx, sy] = toScreen((p.x + next.x) / 2, (p.y + next.y) / 2);
                return <circle key={i} cx={sx} cy={sy} r={2 * k} className={styles.midpoint} />;
              })}

            {selectedArea.outline.map((p, i) => {
              const [sx, sy] = toScreen(p.x, p.y);
              return (
                <circle
                  key={i}
                  cx={sx}
                  cy={sy}
                  r={(i === activeIndex ? 4 : vertexR) * k}
                  className={i === activeIndex ? styles.activeVertex : styles.vertex}
                />
              );
            })}
          </>
        )}

        {overlay?.map((line, i) => {
          if (line.points.length < 2) return null;
          const pts = line.points.map((p) => toScreen(p.x, p.y).join(',')).join(' ');
          const cls = OVERLAY_CLASS[line.color] ?? styles.overlayLive;
          return line.closed ? (
            <polygon key={'ov' + i} points={pts} className={cls} />
          ) : (
            <polyline key={'ov' + i} points={pts} className={cls} />
          );
        })}

        {preview?.map((piece, i) => (
          <polygon
            key={'piece' + i}
            points={piece.map((p) => toScreen(p.x, p.y).join(',')).join(' ')}
            className={[styles.pieceA, styles.pieceB, styles.pieceC][i] ?? styles.pieceB}
          />
        ))}

        {pendingPoints && pendingPoints.length >= 2 && (
          <polyline
            points={pendingPoints.map((p) => toScreen(p.x, p.y).join(',')).join(' ')}
            className={styles.splitLine}
          />
        )}
        {onInsertPending &&
          pendingPoints?.slice(1).map((next, i) => {
            const p = pendingPoints[i];
            if (!showMidpoint(p, next)) return null;
            const [sx, sy] = toScreen((p.x + next.x) / 2, (p.y + next.y) / 2);
            return <circle key={'m' + i} cx={sx} cy={sy} r={2.5 * k} className={styles.midpoint} />;
          })}
        {pendingPoints?.map((p, i) => {
          const [sx, sy] = toScreen(p.x, p.y);
          return <circle key={i} cx={sx} cy={sy} r={5 * k} className={styles.splitPoint} />;
        })}
    </>
  );

  const svg = (
    <svg
      ref={attachSvg}
      className={[styles.svg, zoomable ? styles.zoomable : ''].filter(Boolean).join(' ')}
      viewBox={shown ? `${shown.x} ${shown.y} ${shown.size} ${shown.size}` : `0 0 ${WIDTH} ${HEIGHT}`}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={onPointerEnd}
      onPointerCancel={onPointerEnd}
      onClickCapture={(e) => {
        // swallow the click at the end of a pan/pinch
        if (gesture.current.moved) {
          e.stopPropagation();
          gesture.current.moved = false;
        }
      }}
      onDoubleClick={(e) => {
        const hit = pickHandle(e.clientX, e.clientY, false);
        if (hit && !hit.mid && selectedArea && selectedArea.outline.length > 3) {
          onDeleteVertex?.(selectedArea.id, hit.index);
          setActive(null);
        }
      }}
      onClick={(e) => {
        setActive(null);
        if (!pickingPoints && !(e.target as Element).closest('[data-area]')) onClickEmpty?.();
        if (!pickingPoints || !onCanvasClick) return;
        const local = clientToLocal(e.clientX, e.clientY);
        if (local) onCanvasClick(local[0], local[1]);
      }}
    >
      {renderContent(k)}
      {/* last, so a spot at the dock isn't hidden under the mower */}
      {markers?.map((m, i) => {
        const [mx, my] = toScreen(m.x, m.y);
        return (
          <g key={'mk' + i} className={styles.marker}>
            <circle cx={mx} cy={my} r={20 * k} className={styles.markerPulse} />
            <circle cx={mx} cy={my} r={6 * k} />
          </g>
        );
      })}
    </svg>
  );

  if (!zoomable) return svg;

  // magnified copy of the map above the finger, the finger hides the point otherwise
  let loupe = null;
  const dragged =
    dragging && (dragging.areaId === PENDING ? pendingPoints?.[dragging.index] : selectedArea?.outline[dragging.index]);
  if (finger && dragged) {
    const [cx, cy] = toScreen(dragged.x, dragged.y);
    const size = (LOUPE_PX * (shown?.size ?? WIDTH)) / finger.width / LOUPE_ZOOM;
    const left = Math.min(finger.width - LOUPE_PX, Math.max(0, finger.x - LOUPE_PX / 2));
    // above the finger, or below it when there's no room at the top
    const above = finger.y - LOUPE_PX - 60;
    const top = above >= 0 ? above : Math.min(finger.width - LOUPE_PX, finger.y + 60);
    loupe = (
      <div className={styles.loupe} style={{left, top, width: LOUPE_PX, height: LOUPE_PX}}>
        <svg className={styles.svg} viewBox={`${cx - size / 2} ${cy - size / 2} ${size} ${size}`}>
          {renderContent(k / LOUPE_ZOOM)}
        </svg>
        <span className={styles.crosshair} />
      </div>
    );
  }

  return (
    <div className={styles.wrap}>
      {svg}
      {loupe}
      <MapControls
        onZoom={zoomCenter}
        showGrid={showGrid}
        onToggleGrid={toggleGrid}
        sources={sources}
        source={source}
        imagerySettings={settings.imagery}
        onSource={setImagery}
        hidden={hidden}
        onToggleLayer={toggleLayer}
        layersOpen={layersOpen}
        onLayersOpen={setLayersOpen}
        planStyle={
          progress
            ? {
                value: planStyle,
                onChange: setPlanStyle,
              }
            : undefined
        }
        reset={
          view || restore || (following && followZoom !== 1)
            ? {
                follow: following,
                onReset: () => {
                  setView(null);
                  setRestore(null);
                  setFollowZoom(1);
                  if (viewKey) savedViews.delete(viewKey);
                },
              }
            : null
        }
        follow={
          follow
            ? {
                on: following,
                onToggle: () => {
                  // carries on from what's on screen instead of jumping to the whole map
                  if (following && shown) setView(shown);
                  setFree(following);
                },
              }
            : undefined
        }
      />
      {grid && <span className={styles.gridLabel}>{tr('grid {n} m', {n: grid.step})}</span>}
      {selectedArea && activeIndex !== null && onDeleteVertex && selectedArea.outline.length > 3 && (
        <button
          className={styles.deletePoint}
          onClick={() => {
            onDeleteVertex(selectedArea.id, activeIndex);
            setActive(null);
          }}
        >
          {tr('Delete point')}
        </button>
      )}
    </div>
  );
}
