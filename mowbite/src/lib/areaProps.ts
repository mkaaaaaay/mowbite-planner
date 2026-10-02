import {useEffect, useState} from 'react';
import {RPC} from './openmower';
import {callRpc, methodsUnknown, rpcMethods} from './rpc';

// The area settings the mower keeps in its map (map.area_properties), newer ones are only offered when it does.
// Mowers from before that rpc can't tell: one with mowing.plan knows mowable too (merged the same night), older
// ones none of the new settings.
let known: Promise<Set<string>> | null = null;

export function areaProperties(): Promise<Set<string>> {
  known ??= (async () => {
    const methods = await rpcMethods();
    // no answer yet, the next one asking tries again
    if (methodsUnknown()) known = null;
    if (methods?.has(RPC.areaProperties)) {
      try {
        const list = await callRpc<string[]>(RPC.areaProperties);
        if (Array.isArray(list)) return new Set(list);
      } catch {}
    }
    return new Set(methods?.has(RPC.areaPlan) ? ['mowable'] : []);
  })();
  return known;
}

export function useAreaProperties(): Set<string> {
  const [props, setProps] = useState<Set<string>>(() => new Set());
  useEffect(() => {
    void areaProperties().then(setProps);
  }, []);
  return props;
}
