#!/bin/sh
# the mowing schedule (read by scheduler.sh) and its log. only lines of the known shapes are kept
# the app on another mower may ask too (several mowers in one app)
# after a Status line, busybox httpd only reads that when it comes first
cors() { printf 'Access-Control-Allow-Origin: *\r\n'; }
F=/data/schedule.txt
if [ "$REQUEST_METHOD" = "POST" ]; then
  len=${CONTENT_LENGTH:-0}
  if [ "$len" -le 0 ] || [ "$len" -gt 20000 ]; then
    printf 'Status: 413 Too Large\r\n'
    cors
    printf 'Content-Type: text/plain\r\n\r\ntoo large\n'
    exit 0
  fi
  head -c "$len" | tr -d '\r' | grep -E '^(enabled [01]|skiprain [01]|skipforecast [01]|forecasthours [1-3]|minbattery [0-9]{1,3}|tz [A-Za-z0-9<>+:,./-]{1,60}|pos -?[0-9]{1,3}\.[0-9]{1,2} -?[0-9]{1,3}\.[0-9]{1,2}|plan [1-7](,[1-7]){0,6} [0-2][0-9]:[0-5][0-9]( [A-Za-z0-9_-]{1,40}(,[A-Za-z0-9_-]{1,40}){0,49})?( end=[0-2][0-9]:[0-5][0-9])?( dark=1)?( sunset=1)?( fresh=1)?|pause [0-9]{4}-[0-9]{2}-[0-9]{2} (all|[A-Za-z0-9_-]{1,40}(,[A-Za-z0-9_-]{1,40}){0,49}))$' > "$F.tmp"
  mv "$F.tmp" "$F"
fi
cors
printf 'Content-Type: text/plain\r\nCache-Control: no-store\r\n\r\n'
if [ "$QUERY_STRING" = "log" ]; then
  cat /data/schedule.log 2>/dev/null
else
  cat "$F" 2>/dev/null
fi
