import {toUtm, utmZone} from './utm';

// Official orthophotos, all free to use in apps without a key (licenses checked 2026-09). The German
// states as wms, other countries as web mercator tiles. Saarland isn't here, it needs a contract for
// use in other apps. Anything else the user adds as an xyz tile url in the settings and is
// responsible for its terms.
const YEAR = new Date().getFullYear();

interface WmsSource {
  label: string;
  url: string;
  layer: string;
  // rough lat/lon box of the state, [south, north, west, east]
  box: [number, number, number, number];
  attribution: string;
}

export const STATE_IMAGERY: Record<string, WmsSource> = {
  ni: {
    label: 'Niedersachsen DOP20',
    url: 'https://opendata.lgln.niedersachsen.de/doorman/noauth/dop_wms',
    layer: 'ni_dop20',
    box: [51.29, 53.9, 6.65, 11.6],
    attribution: `LGLN (${YEAR}) CC BY 4.0`,
  },
  nw: {
    label: 'NRW DOP',
    url: 'https://www.wms.nrw.de/geobasis/wms_nw_dop',
    layer: 'nw_dop_rgb',
    box: [50.32, 52.53, 5.86, 9.46],
    attribution: 'Land NRW, dl-de/zero-2-0',
  },
  by: {
    label: 'Bayern DOP40',
    url: 'https://geoservices.bayern.de/od/wms/dop/v1/dop40',
    layer: 'by_dop40c',
    box: [47.27, 50.57, 8.97, 13.84],
    attribution: 'Bayerische Vermessungsverwaltung, CC BY 4.0',
  },
  be: {
    label: 'Berlin TrueDOP',
    url: 'https://gdi.berlin.de/services/wms/truedop_2024',
    layer: 'truedop_2024',
    box: [52.33, 52.68, 13.08, 13.77],
    attribution: 'Geoportal Berlin, dl-de/zero-2-0',
  },
  bb: {
    label: 'Brandenburg DOP20',
    url: 'https://isk.geobasis-bb.de/mapproxy/dop20c/service/wms',
    layer: 'bebb_dop20c',
    box: [51.36, 53.56, 11.26, 14.77],
    attribution: 'GeoBasis-DE/LGB, dl-de/by-2-0',
  },
  hh: {
    label: 'Hamburg DOP',
    url: 'https://geodienste.hamburg.de/wms_dop_zeitreihe_unbelaubt',
    layer: 'dop_zeitreihe_unbelaubt',
    box: [53.39, 53.74, 9.73, 10.33],
    attribution: 'FHH, LGV, dl-de/by-2-0',
  },
  sn: {
    label: 'Sachsen DOP20',
    url: 'https://geodienste.sachsen.de/wms_geosn_dop-rgb/guest',
    layer: 'sn_dop_020',
    box: [50.17, 51.69, 11.87, 15.04],
    attribution: 'GeoSN, dl-de/by-2-0',
  },
  th: {
    label: 'Thüringen DOP20',
    url: 'https://www.geoproxy.geoportal-th.de/geoproxy/services/DOP20',
    layer: 'th_dop',
    box: [50.2, 51.65, 9.87, 12.65],
    attribution: 'GDI-Th, CC BY 4.0',
  },
  he: {
    label: 'Hessen DOP20',
    url: 'https://www.gds-srv.hessen.de/cgi-bin/lika-services/ogc-free-images.ows',
    layer: 'he_dop20_rgb',
    box: [49.39, 51.66, 7.77, 10.24],
    attribution: 'HVBG, dl-de/zero-2-0',
  },
  st: {
    label: 'Sachsen-Anhalt DOP20',
    url: 'https://www.geodatenportal.sachsen-anhalt.de/wss/service/ST_LVermGeo_DOP_WMS_OpenData/guest',
    layer: 'lsa_lvermgeo_dop20_2',
    box: [50.94, 53.05, 10.56, 13.19],
    attribution: 'GeoBasis-DE / LVermGeo ST, dl-de/by-2-0',
  },
  sh: {
    label: 'Schleswig-Holstein DOP20',
    url: 'https://service.gdi-sh.de/WMS_SH_DOP20col_OpenGBD',
    layer: 'sh_dop20_rgb',
    box: [53.36, 55.06, 7.86, 11.32],
    attribution: 'GeoBasis-DE/LVermGeo SH/CC BY 4.0',
  },
  mv: {
    label: 'Mecklenburg-Vorpommern DOP',
    url: 'https://www.geodaten-mv.de/dienste/adv_dop',
    layer: 'mv_dop',
    box: [53.11, 54.69, 10.59, 14.41],
    attribution: `GeoBasis-DE/M-V ${YEAR}`,
  },
  rp: {
    label: 'Rheinland-Pfalz DOP20',
    url: 'https://geo4.service24.rlp.de/wms/rp_dop20.fcgi',
    layer: 'rp_dop20',
    box: [48.97, 50.94, 6.11, 8.51],
    attribution: `GeoBasis-DE / LVermGeoRP (${YEAR}), dl-de/by-2-0`,
  },
  hb: {
    label: 'Bremen DOP20',
    url: 'https://geodienste.bremen.de/wms_dop20_2023',
    layer: 'DOP20_2023_HB',
    box: [53.01, 53.61, 8.48, 8.99],
    attribution: 'Landesamt GeoInformation Bremen, CC BY',
  },
  bw: {
    label: 'Baden-Württemberg DOP20',
    url: 'https://owsproxy.lgl-bw.de/owsproxy/ows/WMS_LGL-BW_ATKIS_DOP_20_C',
    layer: 'IMAGES_DOP_20_RGB',
    box: [47.53, 49.79, 7.51, 10.5],
    attribution: `LGL-BW (${YEAR}) dl-de/by-2-0`,
  },
};

interface TileSource {
  label: string;
  // {z}/{x}/{y}, or {bbox} for a wms that takes EPSG:3857
  template: string;
  // rough lat/lon boxes, [south, north, west, east]
  boxes: [number, number, number, number][];
  attribution: string;
}

export const COUNTRY_IMAGERY: Record<string, TileSource> = {
  at: {
    label: 'Österreich basemap.at',
    template: 'https://maps.wien.gv.at/basemap/bmaporthofoto30cm/normal/google3857/{z}/{y}/{x}.jpeg',
    boxes: [[46.37, 49.02, 9.53, 17.16]],
    attribution: 'Datenquelle: basemap.at, CC BY 4.0',
  },
  ch: {
    label: 'Schweiz SWISSIMAGE',
    template: 'https://wmts.geo.admin.ch/1.0.0/ch.swisstopo.swissimage/default/current/3857/{z}/{x}/{y}.jpeg',
    boxes: [[45.82, 47.81, 5.96, 10.49]],
    attribution: 'swisstopo',
  },
  nl: {
    label: 'Nederland Luchtfoto',
    template: 'https://service.pdok.nl/hwh/luchtfotorgb/wmts/v1_0/Actueel_orthoHR/EPSG:3857/{z}/{x}/{y}.jpeg',
    boxes: [[50.75, 53.56, 3.36, 7.23]],
    attribution: 'Beeldmateriaal.nl, CC BY 4.0',
  },
  vl: {
    label: 'Vlaanderen Orthofoto',
    template:
      'https://geo.api.vlaanderen.be/OMWRGBMRVL/wms?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetMap&LAYERS=Ortho&STYLES=' +
      '&CRS=EPSG:3857&BBOX={bbox}&WIDTH=256&HEIGHT=256&FORMAT=image/jpeg',
    boxes: [[50.68, 51.51, 2.54, 5.92]],
    attribution: 'Bron: Luchtopnamen Digitaal Vlaanderen',
  },
  wa: {
    label: 'Wallonie Orthophotos',
    template:
      'https://geoservices.wallonie.be/arcgis/services/IMAGERIE/ORTHO_LAST/MapServer/WMSServer?SERVICE=WMS&VERSION=1.3.0' +
      '&REQUEST=GetMap&LAYERS=0&STYLES=&CRS=EPSG:3857&BBOX={bbox}&WIDTH=256&HEIGHT=256&FORMAT=image/jpeg',
    boxes: [[49.49, 50.82, 2.84, 6.41]],
    attribution: 'SPW, CC BY 4.0',
  },
  lu: {
    label: 'Luxembourg Ortho',
    template: 'https://wmts1.geoportail.lu/opendata/wmts/ortho_latest/GLOBAL_WEBMERCATOR_4_V3/{z}/{x}/{y}.jpeg',
    boxes: [[49.44, 50.18, 5.73, 6.53]],
    attribution: 'ACT Luxembourg, CC0',
  },
  fr: {
    label: 'France IGN',
    template:
      'https://data.geopf.fr/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=ORTHOIMAGERY.ORTHOPHOTOS&STYLE=normal' +
      '&TILEMATRIXSET=PM&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}&FORMAT=image/jpeg',
    boxes: [[41.33, 51.09, -5.14, 9.56]],
    attribution: 'IGN, Etalab 2.0',
  },
  es: {
    label: 'España PNOA',
    template:
      'https://www.ign.es/wmts/pnoa-ma?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=OI.OrthoimageCoverage&STYLE=default' +
      '&TILEMATRIXSET=GoogleMapsCompatible&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}&FORMAT=image/jpeg',
    boxes: [
      [35.9, 43.8, -9.3, 4.33],
      [27.6, 29.5, -18.2, -13.4],
    ],
    attribution: 'PNOA © IGN España, CC BY 4.0 scne.es',
  },
  cz: {
    label: 'Česko Ortofoto',
    template: 'https://ags.cuzk.cz/arcgis1/rest/services/ORTOFOTO_WM/MapServer/tile/{z}/{y}/{x}',
    boxes: [[48.55, 51.06, 12.09, 18.86]],
    attribution: 'ČÚZK, CC BY 4.0',
  },
  // the national land survey's own service needs a personal key, kapsi (a finnish non-profit) serves
  // the same open data without one
  fi: {
    label: 'Suomi Ortokuva',
    template:
      'https://tiles.kartat.kapsi.fi/ortokuva?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetMap&LAYERS=ortokuva&STYLES=' +
      '&CRS=EPSG:3857&BBOX={bbox}&WIDTH=256&HEIGHT=256&FORMAT=image/jpeg',
    boxes: [[59.45, 70.1, 19.0, 31.6]],
    attribution: 'Maanmittauslaitos Ortokuva 09/2024, CC BY 4.0, kartat.kapsi.fi',
  },
  us: {
    label: 'USA NAIP',
    template:
      'https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPImagery/ImageServer/exportImage' +
      '?bbox={bbox}&bboxSR=3857&imageSR=3857&size=256,256&format=jpg&f=image',
    boxes: [[24.5, 49.4, -124.8, -66.9]],
    attribution: 'USGS / USDA NAIP, public domain',
  },
};

const inBox = (d: {lat: number; lon: number}, [s, n, w, e]: [number, number, number, number]) =>
  d.lat >= s && d.lat <= n && d.lon >= w && d.lon <= e;

// a key from STATE_IMAGERY or COUNTRY_IMAGERY, or the user's own xyz source
export type ImagerySource = string;

export interface CustomImagery {
  url?: string;
  attribution?: string;
}

// xyz tiles with the three placeholders, or a wms in EPSG:3857 with {bbox}. http(s) only
export function isTileUrl(url: string | undefined): url is string {
  return !!url && /^https?:\/\//.test(url) && (url.includes('{bbox}') || ['{z}', '{x}', '{y}'].every((p) => url.includes(p)));
}

export function imageryInfo(source: ImagerySource, custom?: CustomImagery): {label: string; attribution: string} {
  const known = STATE_IMAGERY[source] ?? COUNTRY_IMAGERY[source];
  if (known) return {label: known.label, attribution: known.attribution};
  let host = '';
  try {
    host = new URL(custom?.url ?? '').host;
  } catch {}
  return {label: 'Own source', attribution: custom?.attribution?.trim() || host};
}

export interface Datum {
  lat: number;
  lon: number;
}

// the states and countries whose box contains the mower (neighbours overlap a bit), then the own
// source. the german wms tiles are requested in the map's utm zone, the services only offer 32 and 33
export function availableSources(datum: Datum, custom?: CustomImagery): ImagerySource[] {
  const out: ImagerySource[] = [];
  const zone = utmZone(datum.lat, datum.lon);
  if (zone === 32 || zone === 33) {
    for (const [key, st] of Object.entries(STATE_IMAGERY)) if (inBox(datum, st.box)) out.push(key);
  }
  // smallest box first, near a border the region the mower is in usually has the smaller one
  const area = ([s, n, w, e]: [number, number, number, number]) => (n - s) * (e - w);
  out.push(
    ...Object.entries(COUNTRY_IMAGERY)
      .flatMap(([key, c]) => c.boxes.filter((b) => inBox(datum, b)).map((b) => [key, area(b)] as const))
      .sort((a, b) => a[1] - b[1])
      .map(([key]) => key),
  );
  if (isTileUrl(custom?.url)) out.push('custom');
  return out;
}

// one image on the map, local meters are turned into svg units by the caller
export interface ImageryTile {
  href: string;
  // local map coords of the tile's top left, top right and bottom left corner
  corners: [{x: number; y: number}, {x: number; y: number}, {x: number; y: number}];
}

interface View {
  left: number;
  right: number;
  bottom: number;
  top: number;
  metersPerPixel: number;
}

const MAX_TILES = 120;

// wms tiles on a grid in utm space, so they line up exactly with the map and get cached by the browser
function wmsTiles(src: WmsSource, datum: Datum, v: View): ImageryTile[] {
  const zone = utmZone(datum.lat, datum.lon);
  const d = toUtm(datum.lat, datum.lon, zone);
  const res = 2 ** Math.max(-3, Math.min(5, Math.floor(Math.log2(v.metersPerPixel))));
  const span = 256 * res;
  const x0 = Math.floor((d.e + v.left) / span);
  const x1 = Math.floor((d.e + v.right) / span);
  const y0 = Math.floor((d.n + v.bottom) / span);
  const y1 = Math.floor((d.n + v.top) / span);
  if ((x1 - x0 + 1) * (y1 - y0 + 1) > MAX_TILES) return [];

  const tiles: ImageryTile[] = [];
  for (let ix = x0; ix <= x1; ix++) {
    for (let iy = y0; iy <= y1; iy++) {
      const e = ix * span;
      const n = iy * span;
      const href =
        `${src.url}?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetMap&LAYERS=${src.layer}&STYLES=` +
        `&CRS=EPSG:258${zone}&BBOX=${e},${n},${e + span},${n + span}&WIDTH=256&HEIGHT=256&FORMAT=image/jpeg`;
      const x = e - d.e;
      const y = n - d.n;
      tiles.push({
        href,
        corners: [
          {x, y: y + span},
          {x: x + span, y: y + span},
          {x, y},
        ],
      });
    }
  }
  return tiles;
}

const tileLon = (x: number, z: number) => (x / 2 ** z) * 360 - 180;
const tileLat = (y: number, z: number) => (Math.atan(Math.sinh(Math.PI * (1 - (2 * y) / 2 ** z))) * 180) / Math.PI;

const mercX = (lon: number) => (lon * Math.PI * 6378137) / 180;
const mercY = (lat: number) => 6378137 * Math.log(Math.tan(Math.PI / 4 + (lat * Math.PI) / 360));

// web mercator xyz tiles, each one placed by its corners in utm so the slight rotation is right
function xyzTiles(template: string, datum: Datum, v: View): ImageryTile[] {
  const zone = utmZone(datum.lat, datum.lon);
  const d = toUtm(datum.lat, datum.lon, zone);
  const cosLat = Math.cos((datum.lat * Math.PI) / 180);
  const z = Math.max(3, Math.min(19, Math.round(Math.log2((156543.03 * cosLat) / v.metersPerPixel))));

  // rough lat/lon of the view, only used to pick tiles, one tile of margin covers the error
  const toLat = (y: number) => datum.lat + y / 111320;
  const toLon = (x: number) => datum.lon + x / (111320 * cosLat);
  const tx = (lon: number) => Math.floor(((lon + 180) / 360) * 2 ** z);
  const ty = (lat: number) => {
    const r = (lat * Math.PI) / 180;
    return Math.floor(((1 - Math.log(Math.tan(r) + 1 / Math.cos(r)) / Math.PI) / 2) * 2 ** z);
  };
  const x0 = tx(toLon(v.left)) - 1;
  const x1 = tx(toLon(v.right)) + 1;
  const y0 = ty(toLat(v.top)) - 1;
  const y1 = ty(toLat(v.bottom)) + 1;
  if ((x1 - x0 + 1) * (y1 - y0 + 1) > MAX_TILES) return [];

  const local = (lat: number, lon: number) => {
    const u = toUtm(lat, lon, zone);
    return {x: u.e - d.e, y: u.n - d.n};
  };
  const tiles: ImageryTile[] = [];
  for (let x = x0; x <= x1; x++) {
    for (let y = y0; y <= y1; y++) {
      tiles.push({
        href: template
          .replace('{z}', String(z))
          .replace('{x}', String(x))
          .replace('{y}', String(y))
          .replace('{bbox}', () =>
            [mercX(tileLon(x, z)), mercY(tileLat(y + 1, z)), mercX(tileLon(x + 1, z)), mercY(tileLat(y, z))]
              .map((v) => v.toFixed(2))
              .join(','),
          ),
        corners: [
          local(tileLat(y, z), tileLon(x, z)),
          local(tileLat(y, z), tileLon(x + 1, z)),
          local(tileLat(y + 1, z), tileLon(x, z)),
        ],
      });
    }
  }
  return tiles;
}

export function imageryTiles(source: ImagerySource, datum: Datum, v: View, custom?: CustomImagery): ImageryTile[] {
  const state = STATE_IMAGERY[source];
  if (state) return wmsTiles(state, datum, v);
  const country = COUNTRY_IMAGERY[source];
  if (country) return xyzTiles(country.template, datum, v);
  return source === 'custom' && isTileUrl(custom?.url) ? xyzTiles(custom.url, datum, v) : [];
}
