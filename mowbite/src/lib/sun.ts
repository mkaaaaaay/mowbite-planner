// Sunrise and sunset for a place and a day, the usual NOAA formulas. Good to a minute or two,
// which is plenty for deciding whether it's dark at a start time.

export type SunTimes = {rise: string; set: string};

const rad = (d: number) => (d * Math.PI) / 180;
const deg = (r: number) => (r * 180) / Math.PI;

// minutes after local midnight as "HH:MM"
const clock = (minutes: number) => {
  const m = ((Math.round(minutes) % 1440) + 1440) % 1440;
  return `${String(Math.floor(m / 60)).padStart(2, '0')}:${String(m % 60).padStart(2, '0')}`;
};

// null when the sun doesn't rise or set that day (polar regions)
export function sunTimes(lat: number, lon: number, date = new Date()): SunTimes | null {
  const noonUtc = Date.UTC(date.getFullYear(), date.getMonth(), date.getDate(), 12);
  const julianDay = noonUtc / 86400000 + 2440587.5;
  const t = (julianDay - 2451545) / 36525;

  const meanLon = (280.46646 + t * (36000.76983 + t * 0.0003032)) % 360;
  const meanAnom = 357.52911 + t * (35999.05029 - 0.0001537 * t);
  const ecc = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
  const center =
    Math.sin(rad(meanAnom)) * (1.914602 - t * (0.004817 + 0.000014 * t)) +
    Math.sin(rad(2 * meanAnom)) * (0.019993 - 0.000101 * t) +
    Math.sin(rad(3 * meanAnom)) * 0.000289;
  const trueLon = meanLon + center;
  const omega = 125.04 - 1934.136 * t;
  const apparentLon = trueLon - 0.00569 - 0.00478 * Math.sin(rad(omega));
  const obliquity = 23 + (26 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60) / 60;
  const obliqCorr = obliquity + 0.00256 * Math.cos(rad(omega));
  const declination = deg(Math.asin(Math.sin(rad(obliqCorr)) * Math.sin(rad(apparentLon))));

  const y = Math.tan(rad(obliqCorr / 2)) ** 2;
  const eqTime =
    4 *
    deg(
      y * Math.sin(2 * rad(meanLon)) -
        2 * ecc * Math.sin(rad(meanAnom)) +
        4 * ecc * y * Math.sin(rad(meanAnom)) * Math.cos(2 * rad(meanLon)) -
        0.5 * y * y * Math.sin(4 * rad(meanLon)) -
        1.25 * ecc * ecc * Math.sin(2 * rad(meanAnom)),
    );

  // the sun's centre 0.833° below the horizon, that's refraction plus the disc
  const cosHa =
    (Math.cos(rad(90.833)) - Math.sin(rad(lat)) * Math.sin(rad(declination))) /
    (Math.cos(rad(lat)) * Math.cos(rad(declination)));
  if (cosHa < -1 || cosHa > 1) return null;
  const hourAngle = deg(Math.acos(cosHa));

  const noon = 720 - 4 * lon - eqTime;
  const offset = -date.getTimezoneOffset();
  return {rise: clock(noon - 4 * hourAngle + offset), set: clock(noon + 4 * hourAngle + offset)};
}

export type Season = 'spring' | 'summer' | 'autumn' | 'winter';

// the season as weather services count it, whole months: winter is december to february, south of the equator june
// to august. without a position the north
export function seasonOf(lat?: number, date = new Date()): Season {
  const m = (date.getMonth() + (lat !== undefined && lat < 0 ? 6 : 0)) % 12;
  return m < 2 || m === 11 ? 'winter' : m < 5 ? 'spring' : m < 8 ? 'summer' : 'autumn';
}
