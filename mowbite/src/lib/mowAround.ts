import type {MapArea, Point} from '@/hooks/useMowerMap';
import {shareInside} from './geometry';

// like the mower plans an area: don't mow areas with mow_around that lie in it are left out (an outline pass
// around them, no lanes across). one that covers this whole area doesn't count, the inner area does
export function mowAroundHoles(area: MapArea, areas: MapArea[]): Point[][] {
  return areas
    .filter(
      (o) =>
        o.id !== area.id &&
        o.properties.type === 'mow' &&
        o.properties.active !== false &&
        o.properties.mowable === false &&
        o.properties.mow_around === true &&
        o.outline.length > 2 &&
        shareInside(area.outline, o.outline) < 1,
    )
    .map((o) => o.outline);
}
