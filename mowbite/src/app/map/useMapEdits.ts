import type {MowerMap, Point} from '@/hooks/useMowerMap';
import {useKeptState} from '@/hooks/useKeptState';
import {tr} from '@/lib/i18n';
import {mergeMaps} from '@/lib/mergeMap';
import {sameMap} from '@/lib/sameMap';
import {setUnsavedMap} from '@/lib/unsavedMap';
import {useEffect, useMemo, useState} from 'react';

// The map being edited: the live map until the first edit, after that a copy of our own so updates don't overwrite
// edits, with undo steps. Kept while the app runs, a look at another tab doesn't lose them. Also notices when the
// map on the mower changed meanwhile and can put both together.
export function useMapEdits(liveMap: MowerMap | null) {
  const [edited, setEdited] = useKeptState<MowerMap | null>('map:edited', null);
  // the mower's map the edits started from, to notice when it gets changed somewhere else meanwhile
  const [base, setBase] = useKeptState<MowerMap | null>('map:base', null);
  const map = edited ?? liveMap;
  const setMap = (next: MowerMap | null | ((m: MowerMap | null) => MowerMap | null)) => {
    if (!edited) setBase(liveMap);
    setEdited((prev) => (typeof next === 'function' ? next(prev ?? liveMap) : next));
  };
  const [history, setHistory] = useKeptState<MowerMap[]>('map:history', []);
  // outlines before the first reduce, so the slider can go back up
  const [originals, setOriginals] = useKeptState<Record<string, Point[]>>('map:originals', {});

  // really changed, not just a field that got focus (that already makes an undo step)
  const dirty = useMemo(() => !!edited && (!base || !sameMap(edited, base)), [edited, base]);
  // the map on the mower isn't the one the edits started from: another app, device or a recording changed it.
  // saving would put the edited copy in its place and that change would be gone
  const external = useMemo(() => !!base && !!liveMap && !sameMap(base, liveMap), [base, liveMap]);
  // back to the mower's map. keep: the outlines from before reducing points stay (only after a save, they
  // still match then)
  const dropEdits = (keep?: 'originals') => {
    setEdited(null);
    setBase(null);
    setHistory([]);
    setMergeNote(null);
    if (keep !== 'originals') setOriginals({});
  };
  // what got changed here put together with the change on the mower, see lib/mergeMap. areas both changed keep the
  // version from here, the note names them
  const [mergeNote, setMergeNote] = useState<string | null>(null);
  const mergeExternal = () => {
    if (!edited || !base || !liveMap) return;
    const {map: merged, conflicts} = mergeMaps(base, edited, liveMap);
    remember();
    setEdited(merged);
    setBase(liveMap);
    const name = (id: string) =>
      merged.docking_stations.some((d) => d.id === id)
        ? tr('Docking station')
        : merged.areas.find((a) => a.id === id)?.properties.name || tr('unnamed');
    setMergeNote(conflicts.length ? tr('Changed on both sides: {names}. Your version is kept there.', {names: conflicts.map(name).join(', ')}) : null);
  };
  useEffect(() => setUnsavedMap(dirty), [dirty]);
  // nothing of ours in the copy: follow the mower's map again
  useEffect(() => {
    if (external && !dirty) dropEdits();
    // eslint-disable-next-line react-hooks/exhaustive-deps -- dropEdits only uses setters
  }, [external, dirty]);

  // call before every change so it can be undone. the same map twice in a row is one step (a field that got focus
  // and then the first change after a save)
  const remember = () => {
    if (map) setHistory((h) => (h[h.length - 1] === map ? h : [...h.slice(-49), map]));
  };

  // one step back, false when there's none
  const undoStep = () => {
    if (!history.length) return false;
    setMap(history[history.length - 1]);
    setHistory((h) => h.slice(0, -1));
    return true;
  };

  return {map, edited, setMap, history, remember, undoStep, originals, setOriginals, dirty, external, dropEdits, mergeNote, mergeExternal};
}
