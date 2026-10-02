// WGS84 -> UTM, the same projection OpenMower uses for its map frame (LLtoUTM from robot_localization):
// map x/y = easting/northing minus the datum's easting/northing, in the datum's zone.

const A = 6378137;
const F = 1 / 298.257223563;
const E2 = F * (2 - F);
const EP2 = E2 / (1 - E2);
const K0 = 0.9996;
const DEG = Math.PI / 180;

export function utmZone(lat: number, lon: number): number {
  // the two irregular areas around norway and svalbard
  if (lat >= 56 && lat < 64 && lon >= 3 && lon < 12) return 32;
  if (lat >= 72 && lat < 84) {
    if (lon >= 0 && lon < 9) return 31;
    if (lon >= 9 && lon < 21) return 33;
    if (lon >= 21 && lon < 33) return 35;
    if (lon >= 33 && lon < 42) return 37;
  }
  return Math.floor((lon + 180) / 6) + 1;
}

export function toUtm(lat: number, lon: number, zone: number): {e: number; n: number} {
  const phi = lat * DEG;
  const lam = lon * DEG;
  const lam0 = ((zone - 1) * 6 - 180 + 3) * DEG;

  const sin = Math.sin(phi);
  const cos = Math.cos(phi);
  const tan = Math.tan(phi);
  const N = A / Math.sqrt(1 - E2 * sin * sin);
  const T = tan * tan;
  const C = EP2 * cos * cos;
  const a = cos * (lam - lam0);
  const e4 = E2 * E2;
  const e6 = e4 * E2;
  const M =
    A *
    ((1 - E2 / 4 - (3 * e4) / 64 - (5 * e6) / 256) * phi -
      ((3 * E2) / 8 + (3 * e4) / 32 + (45 * e6) / 1024) * Math.sin(2 * phi) +
      ((15 * e4) / 256 + (45 * e6) / 1024) * Math.sin(4 * phi) -
      ((35 * e6) / 3072) * Math.sin(6 * phi));

  const e =
    K0 * N * (a + ((1 - T + C) * a ** 3) / 6 + ((5 - 18 * T + T * T + 72 * C - 58 * EP2) * a ** 5) / 120) + 500000;
  let n =
    K0 *
    (M +
      N *
        tan *
        ((a * a) / 2 +
          ((5 - T + 9 * C + 4 * C * C) * a ** 4) / 24 +
          ((61 - 58 * T + T * T + 600 * C - 330 * EP2) * a ** 6) / 720));
  if (lat < 0) n += 10000000;
  return {e, n};
}
