'use client';

import {useEffect, useState} from 'react';

// Open-Meteo (free, no key, CC BY 4.0). Only asked when switched on in the settings, with the
// position rounded to about a kilometer.
export interface Weather {
  temp: number;
  code: number;
  day: boolean;
  max: number;
  min: number;
  raining: boolean;
  // unix seconds of the first hour in the next 6 with rain, if any
  rainAt?: number;
  // rain within half an hour (as of the last update, it comes every 20 min)
  rainSoon: boolean;
}

const EVERY_MS = 20 * 60 * 1000;
let cache: {key: string; at: number; data: Weather} | null = null;

async function load(lat: number, lon: number): Promise<Weather> {
  const url =
    `https://api.open-meteo.com/v1/forecast?latitude=${lat}&longitude=${lon}` +
    '&current=temperature_2m,weather_code,is_day,precipitation' +
    '&hourly=precipitation,precipitation_probability&daily=temperature_2m_max,temperature_2m_min' +
    '&forecast_days=2&timezone=auto&timeformat=unixtime';
  const res = await fetch(url);
  if (!res.ok) throw new Error(`weather ${res.status}`);
  const d = await res.json();
  const now = Date.now() / 1000;
  const h = d.hourly;
  let rainAt: number | undefined;
  for (let i = 0; i < h.time.length; i++) {
    if (h.time[i] + 3600 < now || h.time[i] > now + 6 * 3600) continue;
    if (h.precipitation[i] >= 0.2 || h.precipitation_probability[i] >= 60) {
      rainAt = Math.max(h.time[i], now);
      break;
    }
  }
  const code = d.current.weather_code;
  return {
    temp: d.current.temperature_2m,
    code,
    day: !!d.current.is_day,
    max: d.daily.temperature_2m_max[0],
    min: d.daily.temperature_2m_min[0],
    raining: d.current.precipitation > 0 || (code >= 51 && code <= 67) || (code >= 80 && code <= 82) || code >= 95,
    rainAt,
    rainSoon: rainAt !== undefined && rainAt - now < 1800,
  };
}

export function useWeather(pos: {lat: number; lon: number} | undefined, enabled: boolean): Weather | null {
  const lat = pos ? Math.round(pos.lat * 100) / 100 : undefined;
  const lon = pos ? Math.round(pos.lon * 100) / 100 : undefined;
  const key = `${lat},${lon}`;
  const [data, setData] = useState<Weather | null>(cache?.key === key ? cache.data : null);

  useEffect(() => {
    if (!enabled || lat === undefined || lon === undefined) return;
    let alive = true;
    const run = () => {
      if (cache?.key === key && Date.now() - cache.at < EVERY_MS) {
        setData(cache.data);
        return;
      }
      load(lat, lon).then(
        (w) => {
          cache = {key, at: Date.now(), data: w};
          if (alive) setData(w);
        },
        // no internet or the service is down: the weather just isn't shown
        () => alive && setData(null),
      );
    };
    run();
    const timer = setInterval(run, EVERY_MS);
    return () => {
      alive = false;
      clearInterval(timer);
    };
  }, [enabled, key, lat, lon]);

  return enabled ? data : null;
}
