'use client';

import {TitleMark} from '@/components/Logo';
import MapView from '@/components/MapView';
import TrackPicker from '@/components/TrackPicker';
import {saveMap, useMowerMap, type MowerMap, type Point} from '@/hooks/useMowerMap';
import {useMowerSensors} from '@/hooks/useMowerSensors';
import {useMowerPosition} from '@/hooks/useMowerPosition';
import {useMowerState} from '@/hooks/useMowerState';
import {clearTrack, trackPoints, useMowerTrack} from '@/hooks/useMowerTrack';
import {datumFromParams, numParam, useMowerParams} from '@/hooks/useMowerParams';
import {loadJobTrack, useJobList, useMowHistory, type TrackSegment} from '@/hooks/useMowHistory';
import {simplifyPolygon} from '@/lib/simplifyPolygon';
import {polygonArea, shareInside} from '@/lib/geometry';
import {mergeOutlines} from '@/lib/mergeAreas';
import {cutOut, generateId, splitByPath} from '@/lib/splitPolygon';
import {isDocked} from '@/lib/status';
import {useSearchParams} from 'next/navigation';
import {Suspense, useEffect, useMemo, useState} from 'react';
import styles from './page.module.css';
import {tr, useLang} from '@/lib/i18n';
import MapBackups, {backupLabel} from '@/components/MapBackups';
import {deleteBackup, listBackups, loadBackup, saveBackup, type BackupInfo} from '@/lib/backups';
import AreaCard from './AreaCard';
import {type UpdateArea, NEW_AREA_SETTINGS} from './editing';
import EditorToolbar from './EditorToolbar';
import MowSettings, {AngleOnMap} from './MowSettings';
import OrderBox from './OrderBox';
import {DrawPanel, MergePanel, RestorePanel, SimplifyPanel, SplitPanel} from './Panels';
import {PARAM} from '@/lib/openmower';
import {rpcErrorText} from '@/lib/rpcText';
import {closedRings} from '@/lib/rings';
import {saveFile} from '@/lib/saveFile';
import {useAreaProperties} from '@/lib/areaProps';
import {useMowPlan} from './useMowPlan';
import {useMapEdits} from './useMapEdits';
import {checkMap} from '@/lib/mapCheck';
import Problems from './Problems';

// useSearchParams needs a suspense boundary in a static export
export default function MapPage() {
  return (
    <Suspense>
      <MapEditor />
    </Suspense>
  );
}

function MapEditor() {
  useLang();
  const {state} = useMowerState();
  const position = useMowerPosition() ?? state?.pose;
  const {values: sensorValues} = useMowerSensors();
  const docked = isDocked(state, sensorValues['om_v_charge']);
  const track = useMowerTrack();
  const liveMap = useMowerMap();
  const params = useMowerParams();
  const {map, edited, setMap, history, remember, undoStep, originals, setOriginals, dirty, external, dropEdits, mergeNote, mergeExternal} =
    useMapEdits(liveMap);
  const [showStripes, setShowStripes] = useState(true);
  const pastJobs = useMowHistory();
  const jobList = useJobList();
  // null = live trail, otherwise a recorded job shown instead
  // ?job=<id> (from the activity page) opens that job's track until another one is picked
  const search = useSearchParams();
  const urlJob = search.get('job');
  // ?at=x,y&msg=... from a problem on the activity page: marked on the map with its message
  const at = search.get('at')?.split(',').map(Number);
  const [spotClosed, setSpotClosed] = useState(false);
  const spot = !spotClosed && at?.length === 2 && at.every(Number.isFinite) ? {x: at[0], y: at[1], msg: search.get('msg') ?? ''} : null;
  const [picked, setPicked] = useState<string | null | undefined>(undefined);
  const jobId = picked === undefined ? urlJob : picked;
  const [loaded, setLoaded] = useState<{id: string; segments: TrackSegment[]} | null>(null);
  const viewJob = jobId ? {id: jobId, segments: loaded?.id === jobId ? loaded.segments : null} : null;
  const showJob = (id: string) => setPicked(id || null);
  const loadingJob = viewJob && !viewJob.segments ? viewJob.id : null;
  useEffect(() => {
    if (!loadingJob) return;
    void loadJobTrack(loadingJob).then((segments) => setLoaded({id: loadingJob, segments}));
  }, [loadingJob]);
  // extra rotation for the preview when the mower drives differently than calculated
  const [previewCorrection, setPreviewCorrection] = useState(0);

  const [selectedAreaId, setSelectedAreaId] = useState<string | null>(null);
  const [saving, setSaving] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);
  const [saveWarning, setSaveWarning] = useState(false);
  const [mode, setMode] = useState<'idle' | 'split' | 'draw' | 'merge'>('idle');
  const [mergeWithId, setMergeWithId] = useState<string | null>(null);
  const [pendingPoints, setPendingPoints] = useState<Point[]>([]);
  // split: the points are a closed shape inside the area to cut out, not a line across it
  const [cutShape, setCutShape] = useState(false);
  // tolerance in cm while the simplify preview is open
  const [simplifyCm, setSimplifyCm] = useState<number | null>(null);
  // delete needs a second click, id of the area that's armed
  const [confirmDelete, setConfirmDelete] = useState<string | null>(null);


  // the area settings this mower keeps, mow_around changes the plan
  const areaProps = useAreaProperties();
  const selectedArea = map?.areas.find((a) => a.id === selectedAreaId) ?? null;
  // an active mowing or navigation area the selected one mostly lies in, the mower drives and mows that as a
  // whole, whatever is set on the smaller one. the innermost one when areas are nested deeper
  const enclosing = useMemo(() => {
    const t = selectedArea?.properties.type;
    if (!map || !selectedArea || (t !== 'mow' && t !== 'nav')) return undefined;
    const size = polygonArea(selectedArea.outline);
    return map.areas
      .filter(
        (o) =>
          o.id !== selectedArea.id &&
          o.properties.active !== false &&
          (o.properties.type === 'mow' || o.properties.type === 'nav') &&
          polygonArea(o.outline) > size &&
          shareInside(selectedArea.outline, o.outline) > 0.5,
      )
      .sort((a, b) => polygonArea(a.outline) - polygonArea(b.outline))[0];
  }, [map, selectedArea]);
  const cutFromEnclosing = useMemo(
    () => (enclosing && selectedArea ? cutOut(enclosing.outline, selectedArea.outline) : null),
    [enclosing, selectedArea],
  );
  const applyCutFromEnclosing = () => {
    if (!map || !selectedArea || !enclosing || !cutFromEnclosing) return;
    const [a, b] = cutFromEnclosing;
    const name = enclosing.properties.name;
    // the first half keeps the id, schedules and pauses pick areas by it
    const halves = [a, b].map((outline, i) => ({
      ...enclosing,
      id: i ? generateId() : enclosing.id,
      outline,
      properties: {...enclosing.properties, name: name && i ? `${name} 2` : name},
    }));
    remember();
    setMap({...map, areas: map.areas.flatMap((x) => (x.id === enclosing.id ? halves : [x]))});
  };
  const baseOutline = selectedArea ? (originals[selectedArea.id] ?? selectedArea.outline) : null;
  const simplified = useMemo(
    () =>
      baseOutline && simplifyCm !== null
        ? simplifyCm === 0
          ? baseOutline
          : simplifyPolygon(baseOutline, simplifyCm / 100)
        : null,
    [baseOutline, simplifyCm],
  );
  // kept the same object between renders, the plan below is only worked out again when it changes
  const shownMap = useMemo(
    () =>
      map && selectedArea && simplified
        ? {...map, areas: map.areas.map((a) => (a.id === selectedArea.id ? {...a, outline: simplified} : a))}
        : map,
    [map, selectedArea, simplified],
  );

  // self crossing outlines, a dock off the drivable areas and the like, also in the preview of reducing points
  const approachDistance = numParam(params, PARAM.dockingApproachDistance);
  const problems = useMemo(() => (shownMap ? checkMap(shownMap, approachDistance) : []), [shownMap, approachDistance]);
  const warnings = problems.filter((p) => p.level === 'warn').length;
  const problemSpots = problems.flatMap((p) => ('at' in p ? [p.at] : []));

  // not while a point is dragged, a new plan redrawn mid-drag makes it stutter
  const [draggingPoint, setDraggingPoint] = useState(false);
  const {toolWidth, angleOffset, offsetIsAbsolute, angleIncrement, shownArea, autoAngle, touchAngle, mismatch, realPlan, plan, stripes, planLength} =
    useMowPlan({params, liveMap, shownMap, selectedAreaId, pastJobs, areaProps, showStripes, previewCorrection, draggingPoint});

  const globalValue = (key: string) => {
    const v = numParam(params, PARAM.mowerLogic(key));
    return v === undefined ? 'global' : `global ${v}`;
  };

  const selectArea = (id: string) => {
    if (mode === 'merge') {
      if (id !== selectedAreaId) setMergeWithId(id);
      return;
    }
    setSelectedAreaId(id);
    setSimplifyCm(null);
  };

  const deselect = () => {
    if (mode !== 'idle') return;
    setSelectedAreaId(null);
    setSimplifyCm(null);
  };

  const undo = () => {
    if (!undoStep()) return;
    setSimplifyCm(null);
    setMode('idle');
    setPendingPoints([]);
  };

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if ((e.ctrlKey || e.metaKey) && e.key === 'z') {
        e.preventDefault();
        undo();
      }
      if (e.key === 'Escape' && !(e.target instanceof HTMLInputElement)) {
        if (mode !== 'idle') cancelPicking();
        else deselect();
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  });

  const applySimplify = () => {
    if (shownMap && selectedArea) {
      remember();
      if (!originals[selectedArea.id]) setOriginals({...originals, [selectedArea.id]: selectedArea.outline});
      setMap(shownMap);
    }
    setSimplifyCm(null);
  };

  const updateProperties: UpdateArea = (patch, undoable = true) => {
    if (!map || !selectedArea) return;
    // typing on in a field that kept focus through a save: the first change still gets its undo step
    if (undoable || !edited) remember();
    setMap({
      ...map,
      areas: map.areas.map((a) => (a.id === selectedArea.id ? {...a, properties: {...a.properties, ...patch}} : a)),
    });
  };

  const deleteArea = () => {
    if (!map || !selectedArea) return;
    if (confirmDelete !== selectedArea.id) {
      setConfirmDelete(selectedArea.id);
      return;
    }
    remember();
    setMap({...map, areas: map.areas.filter((a) => a.id !== selectedArea.id)});
    setSelectedAreaId(null);
    setConfirmDelete(null);
  };

  // the mower goes through the mow areas in the order they have in map.json
  const mowAreas = map?.areas.filter((a) => a.properties.type === 'mow') ?? [];

  const moveInOrder = (id: string, by: number) => {
    if (!map || !docked) return;
    const order = [...mowAreas];
    const i = order.findIndex((a) => a.id === id);
    const j = i + by;
    if (i < 0 || j < 0 || j >= order.length) return;
    [order[i], order[j]] = [order[j], order[i]];
    // only the mow areas swap places, everything else stays where it is
    const slots = map.areas.flatMap((a, k) => (a.properties.type === 'mow' ? [k] : []));
    const areas = [...map.areas];
    slots.forEach((slot, k) => (areas[slot] = order[k]));
    remember();
    setMap({...map, areas});
  };


  const moveVertex = (areaId: string, vertexIndex: number, x: number, y: number) => {
    setMap((prev) =>
      prev
        ? {
            ...prev,
            areas: prev.areas.map((a) =>
              a.id === areaId ? {...a, outline: a.outline.map((p, i) => (i === vertexIndex ? {x, y} : p))} : a,
            ),
          }
        : prev,
    );
  };

  const insertVertex = (areaId: string, vertexIndex: number, x: number, y: number) => {
    remember();
    setMap((prev) =>
      prev
        ? {
            ...prev,
            areas: prev.areas.map((a) =>
              a.id === areaId ? {...a, outline: a.outline.toSpliced(vertexIndex, 0, {x, y})} : a,
            ),
          }
        : prev,
    );
  };

  const deleteVertex = (areaId: string, vertexIndex: number) => {
    remember();
    setMap((prev) =>
      prev
        ? {
            ...prev,
            areas: prev.areas.map((a) => (a.id === areaId ? {...a, outline: a.outline.toSpliced(vertexIndex, 1)} : a)),
          }
        : prev,
    );
  };

  const startSplit = () => {
    setSimplifyCm(null);
    setMode('split');
    setPendingPoints([]);
  };

  const startDraw = () => {
    setSelectedAreaId(null);
    setSimplifyCm(null);
    setMode('draw');
    setPendingPoints([]);
  };

  function cancelPicking() {
    setMode('idle');
    setPendingPoints([]);
    setMergeWithId(null);
  }

  const mergeWith = map?.areas.find((a) => a.id === mergeWithId) ?? null;
  const merged = mode === 'merge' && selectedArea && mergeWith ? mergeOutlines(selectedArea.outline, mergeWith.outline) : null;

  const applyMerge = () => {
    if (!map || !selectedArea || !mergeWith || !merged) return;
    // keeps the id of the selected one like its name and settings, schedules and pauses pick areas by it
    const area = {...selectedArea, outline: merged.outline};
    // in the place of the first of the two, the order in map.json is the mowing order
    const first = map.areas.find((a) => a.id === selectedArea.id || a.id === mergeWith.id)!;
    remember();
    setMap({
      ...map,
      areas: map.areas.flatMap((a) => (a === first ? [area] : a.id === selectedArea.id || a.id === mergeWith.id ? [] : [a])),
    });
    setSelectedAreaId(area.id);
    cancelPicking();
  };

  const finishDraw = () => {
    if (!map || pendingPoints.length < 3) return;
    const newArea = {
      id: generateId(),
      properties: {type: 'mow', active: true},
      outline: pendingPoints,
    };
    remember();
    setMap({...map, areas: [...map.areas, newArea]});
    setSelectedAreaId(newArea.id);
    setMode('idle');
    setPendingPoints([]);
  };

  const handleCanvasClick = (x: number, y: number) => {
    if (mode === 'draw') {
      setPendingPoints((prev) => [...prev, {x, y}]);
      return;
    }

    if (mode === 'split') setPendingPoints((prev) => [...prev, {x, y}]);
  };

  const splitPreview: Point[][] | null =
    mode === 'split' && selectedArea
      ? cutShape
        ? cutOut(selectedArea.outline, pendingPoints)
        : splitByPath(selectedArea.outline, pendingPoints)
      : null;

  const applySplit = () => {
    if (!map || !selectedArea || !splitPreview) return;
    const name = selectedArea.properties.name;
    // the first piece keeps the name and the id (schedules and pauses pick areas by it), the others get a number
    const pieces = splitPreview.map((outline, i) => ({
      ...selectedArea,
      id: i ? generateId() : selectedArea.id,
      outline,
      properties: {...selectedArea.properties, name: name && i ? `${name} ${i + 1}` : name},
    }));
    remember();
    setMap({
      ...map,
      areas: map.areas.flatMap((a) => (a.id === selectedArea.id ? pieces : [a])),
    });
    // a cut out shape is the new one, likely to be renamed next
    setSelectedAreaId(pieces[pieces.length - 1 - (cutShape ? 0 : 1)].id);
    setMode('idle');
    setPendingPoints([]);
  };

  // backups of the map kept in the container, and one shown instead of the editor
  const [backups, setBackups] = useState<BackupInfo[] | null | undefined>(undefined);
  const [preview, setPreview] = useState<{map: MowerMap; label: string} | null>(null);
  const [confirmRestore, setConfirmRestore] = useState(false);
  const [restoring, setRestoring] = useState(false);
  const [restoreError, setRestoreError] = useState<string | null>(null);
  const refreshBackups = () => listBackups().then(setBackups);
  useEffect(() => {
    void listBackups().then(setBackups);
  }, []);
  const closePreview = () => {
    setPreview(null);
    setConfirmRestore(false);
    setRestoreError(null);
  };
  const restore = async () => {
    if (!preview) return;
    setRestoring(true);
    setRestoreError(null);
    try {
      if (backups && liveMap) await saveBackup(liveMap, 'before restoring', true);
      await saveMap(preview.map);
      dropEdits();
      setSelectedAreaId(null);
      closePreview();
      void refreshBackups();
    } catch (e) {
      setRestoreError(rpcErrorText(e));
    } finally {
      setRestoring(false);
      setConfirmRestore(false);
    }
  };
  // a backup, or the map as it is on the mower right now, as a file
  const download = async (b: BackupInfo | null) => {
    const m = b ? await loadBackup(b.id) : liveMap;
    if (!m) return;
    const t = new Date((b ? b.t : Date.now() / 1000) * 1000);
    const pad = (n: number) => String(n).padStart(2, '0');
    const name = `mowbite-map-${t.getFullYear()}-${pad(t.getMonth() + 1)}-${pad(t.getDate())}-${pad(t.getHours())}${pad(t.getMinutes())}.json`;
    await saveFile(name, JSON.stringify(closedRings(m), null, 2));
  };

  // an openmower that doesn't know the newer area settings drops them when saving, noticed once its map comes back
  const [newSaved, setNewSaved] = useState<{areas: MowerMap['areas']; before: MowerMap | null} | null>(null);
  const dropped = useMemo(() => {
    if (!newSaved || !liveMap || liveMap === newSaved.before) return [];
    const lost = new Set<string>();
    for (const a of newSaved.areas) {
      const now = liveMap.areas.find((x) => x.id === a.id);
      if (!now) continue;
      for (const k of NEW_AREA_SETTINGS) if (a.properties[k] !== undefined && now.properties[k] === undefined) lost.add(k);
    }
    return [...lost];
  }, [newSaved, liveMap]);
  const droppedNote = dropped.length
    ? tr("Your OpenMower version doesn't know {what} yet, it got dropped when saving.", {
        what: [...new Set(dropped.map((k) => `"${tr(k === 'mowable' ? "don't mow" : 'angle range')}"`))].join(', '),
      })
    : null;

  const handleSave = async () => {
    if (!map) return;
    // saving while the mower is out can make it lose track of the area it's on, ask first
    if (!docked && !saveWarning) {
      setSaveWarning(true);
      return;
    }
    setSaveWarning(false);
    setSaving(true);
    setSaveError(null);
    try {
      // the version on the mower goes into the backups first, so every save can be undone
      if (backups && liveMap) await saveBackup(liveMap, 'before saving', true).catch(() => {});
      await saveMap(map);
      setNewSaved({areas: map.areas.filter((a) => NEW_AREA_SETTINGS.some((k) => a.properties[k] !== undefined)), before: liveMap});
      // back to the live map, it comes back from the mower with what was just saved
      dropEdits('originals');
      void refreshBackups();
    } catch (e) {
      setSaveError(rpcErrorText(e));
    } finally {
      setSaving(false);
    }
  };

  return (
    <div className={styles.page}>
      <main className={styles.main}>
        <h1>
          <TitleMark />
          {tr('Map')}
        </h1>

        <div className={styles.editor}>
          <div className={styles.mapCol}>
            {spot && (
              <div className={styles.spot}>
                <span>{spot.msg || tr('Marked spot')}</span>
                <button onClick={() => setSpotClosed(true)} aria-label="close">
                  ×
                </button>
              </div>
            )}
            {preview && (
              <div className={styles.previewBar}>
                <span>{tr('Preview: {what}', {what: preview.label})}</span>
                <button onClick={closePreview} aria-label="close">
                  ×
                </button>
              </div>
            )}
            {preview && <MapView zoomable map={preview.map} mower={position} datum={datumFromParams(params)} />}
            {!preview && shownMap && (
              <MapView
                zoomable
                viewKey="editor"
                map={shownMap}
                mower={position}
                emergency={!!state?.emergency}
                track={viewJob ? undefined : track}
                pastTrack={viewJob?.segments?.map((s) => ({
                  points: s.points.map(([x, y]) => ({x, y})),
                  blades: !!s.attributes.blades,
                }))}
                selectedAreaId={selectedAreaId}
                onSelectArea={selectArea}
                onMoveVertex={simplifyCm === null && mode === 'idle' ? moveVertex : undefined}
                onDragStart={remember}
                onDragging={setDraggingPoint}
                onDragCancel={undo}
                onInsertVertex={simplifyCm === null ? insertVertex : undefined}
                onDeleteVertex={simplifyCm === null ? deleteVertex : undefined}
                pickingPoints={mode === 'split' || mode === 'draw'}
                onClickEmpty={deselect}
                datum={datumFromParams(params)}
                orderLabels={
                  mowAreas.length > 1 && !selectedArea
                    ? Object.fromEntries(mowAreas.map((a, i) => [a.id, i + 1]))
                    : undefined
                }
                pendingPoints={pendingPoints}
                onCanvasClick={handleCanvasClick}
                onMovePending={(i, x, y) => setPendingPoints((prev) => prev.map((p, j) => (j === i ? {x, y} : p)))}
                onInsertPending={(i, x, y) => setPendingPoints((prev) => prev.toSpliced(i, 0, {x, y}))}
                stripes={stripes}
                loops={plan?.loops}
                preview={splitPreview ?? (merged ? [merged.outline] : undefined)}
                markers={spot || problemSpots.length ? [...(spot ? [spot] : []), ...problemSpots] : undefined}
                focus={spot ?? undefined}
              />
            )}

            {!preview && selectedArea && mode === 'idle' && simplifyCm === null && selectedArea.properties.type === 'mow' && (
              <AngleOnMap area={selectedArea} autoAngle={autoAngle} remember={remember} update={updateProperties} onEdit={touchAngle} />
            )}


            {!map && <p className={styles.dim}>{tr('waiting for map…')}</p>}
          </div>

          <div className={styles.panel}>
            {preview ? (
              <RestorePanel
                areas={preview.map.areas.length}
                backedUp={!!backups}
                docked={docked}
                restoring={restoring}
                confirm={confirmRestore}
                error={restoreError}
                onRestore={() => (confirmRestore ? void restore() : setConfirmRestore(true))}
                onDisarm={() => setConfirmRestore(false)}
                onClose={closePreview}
              />
            ) : (
              <>
              {map && (
                <EditorToolbar
                  idle={mode === 'idle'}
                  canUndo={history.length > 0}
                  unsaved={dirty}
                  saving={saving}
                  saveLabel={
                    saving ? tr('saving…') : (external && dirty) || warnings || (saveWarning && !docked) ? tr('Save anyway') : tr('Save map')
                  }
                  saveError={saveError ?? droppedNote}
                  onDraw={startDraw}
                  onUndo={undo}
                  onSave={() => void handleSave()}
                />
              )}


              {mode === 'idle' && map && <Problems problems={problems} map={map} onSelect={selectArea} />}

              {external && dirty && (
                <div className={styles.warning}>
                  <p>
                    {tr(
                      'The map on the mower was changed since you started editing here (another app or device, or a recording). Saving puts your version in its place.',
                    )}{' '}
                    {/* the save backs up the mower's map first when backups work */}
                    {backups ? tr('That change is then only in the backups.') : tr('That change would be lost.')}
                  </p>
                  <a onClick={mergeExternal}>{tr('put both together (what only one side changed is kept)')}</a>
                  <a
                    onClick={() => {
                      dropEdits();
                      setSelectedAreaId(null);
                    }}
                  >
                    {tr('drop my changes and load it')}
                  </a>
                </div>
              )}

              {mergeNote && dirty && <p className={styles.warning}>{mergeNote}</p>}

              {saveWarning && !docked && (
                <div className={styles.warning}>
                  <p>
                    {tr("The mower isn't idle in the dock. Changing the map during a job can stop the job. Better save once it's back in the dock.")}
                  </p>
                  <a onClick={() => setSaveWarning(false)}>{tr("don't save for now")}</a>
                </div>
              )}

              {map && !selectedArea && mode === 'idle' && (
                <>
                  <p className={styles.dim}>{tr('Click an area to edit it.')}</p>
                  {jobList && jobList.length > 0 && (
                    <TrackPicker
                      jobs={jobList}
                      selected={viewJob?.id ?? null}
                      segments={viewJob?.segments}
                      onSelect={(id) => showJob(id ?? '')}
                      onClearLive={trackPoints(track) > 1 ? () => void clearTrack() : undefined}
                    />
                  )}
                  {mowAreas.length > 1 && (
                    <OrderBox areas={mowAreas} docked={docked} onSelect={selectArea} onMove={moveInOrder} />
                  )}

                  <MapBackups
                    backups={backups ?? null}
                    onCreate={async (name) => {
                      if (!liveMap) return;
                      await saveBackup(liveMap, name, false);
                      await refreshBackups();
                    }}
                    onPreview={(b) =>
                      void loadBackup(b.id).then(
                        (m) => setPreview({map: m, label: backupLabel(b)}),
                        () => setRestoreError(tr('failed')),
                      )
                    }
                    onDelete={(b) => void deleteBackup(b.id).then(refreshBackups)}
                    onDownload={(b) => void download(b)}
                    onFile={(m, name) => setPreview({map: m, label: name})}
                  />
                </>
              )}

              {mode === 'split' && (
                <SplitPanel
                  preview={splitPreview}
                  cutShape={cutShape}
                  onCutShape={setCutShape}
                  points={pendingPoints.length}
                  onApply={applySplit}
                  onRemoveLast={() => setPendingPoints(pendingPoints.slice(0, -1))}
                  onCancel={cancelPicking}
                />
              )}

              {mode === 'merge' && selectedArea && (
                <MergePanel area={selectedArea} other={mergeWith} merged={merged} onApply={applyMerge} onCancel={cancelPicking} />
              )}

              {mode === 'draw' && <DrawPanel points={pendingPoints.length} onFinish={finishDraw} onCancel={cancelPicking} />}

              {selectedArea && mode === 'idle' && (
                <AreaCard
                  area={selectedArea}
                  enclosing={enclosing}
                  onCutOut={cutFromEnclosing && applyCutFromEnclosing}
                  showTools={simplifyCm === null}
                  confirmDelete={confirmDelete === selectedArea.id}
                  remember={remember}
                  update={updateProperties}
                  onSplit={startSplit}
                  onMerge={() => {
                    setSimplifyCm(null);
                    setMode('merge');
                  }}
                  onSimplify={() => setSimplifyCm(5)}
                  onDelete={deleteArea}
                  onDeleteBlur={() => setConfirmDelete(null)}
                />
              )}

              {selectedArea && mode === 'idle' && simplifyCm === null && selectedArea.properties.type === 'mow' && (
                <MowSettings
                  area={selectedArea}
                  autoAngle={autoAngle}
                  globalValue={globalValue}
                  remember={remember}
                  update={updateProperties}
                  showStripes={showStripes}
                  onToggleStripes={() => setShowStripes(!showStripes)}
                  onAngleEdit={touchAngle}
                  toolWidth={toolWidth}
                  mismatch={mismatch}
                  previewCorrection={previewCorrection}
                  planFromMower={!!realPlan}
                  planAngle={realPlan?.angle}
                  planLength={shownArea?.properties.mowable === false || shownArea?.properties.active === false ? 0 : planLength}
                  onPreviewCorrection={setPreviewCorrection}
                  angle={{offset: angleOffset, offsetIsAbsolute, increment: angleIncrement}}
                />
              )}

              {selectedArea && simplified && simplifyCm !== null && (
                <SimplifyPanel
                  cm={simplifyCm}
                  from={baseOutline?.length ?? 0}
                  to={simplified.length}
                  onChange={setSimplifyCm}
                  onApply={applySimplify}
                  onCancel={() => setSimplifyCm(null)}
                />
              )}

              </>
            )}
          </div>
        </div>
      </main>
    </div>
  );
}
