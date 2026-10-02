#!/bin/sh
# the ros warnings and errors logkeeper.sh kept, around a moment: ?t=<unix time>&before=<s>&after=<s>
# 404 when nothing is kept (older container, or OpenMower without logs.recent)
# the app on another mower may ask too, after a Status line: busybox httpd only reads that when it comes first
cors() { printf 'Access-Control-Allow-Origin: *\r\n'; }
D=/data/roslog
if [ ! -f "$D/.last" ]; then
  printf 'Status: 404 Not Found\r\n'
  cors
  printf 'Content-Type: text/plain\r\n\r\n'
  exit 0
fi
# ?oldest: the time of the oldest line kept, the activity page only offers the log for problems after that
if [ "$QUERY_STRING" = oldest ]; then
  cors
  printf 'Content-Type: text/plain\r\nCache-Control: no-store\r\n\r\n'
  f=$(ls "$D"/*.log 2>/dev/null | head -n1)
  [ -n "$f" ] && head -n1 "$f" | cut -d' ' -f1 | cut -d. -f1
  exit 0
fi
num() { printf '%s' "$QUERY_STRING" | tr '&' '\n' | sed -n "s/^$1=\([0-9]*\)$/\1/p" | head -n1; }
t=$(num t)
before=$(num before)
after=$(num after)
from=$((${t:-0} - ${before:-120}))
to=$((${t:-0} + ${after:-60}))
cors
printf 'Content-Type: text/plain\r\nCache-Control: no-store\r\n\r\n'
# the day files the range touches
for f in $(awk -v a="$from" -v b="$to" 'BEGIN { print strftime("%Y%m%d", a); if (strftime("%Y%m%d", b) != strftime("%Y%m%d", a)) print strftime("%Y%m%d", b) }'); do
  [ -f "$D/$f.log" ] && awk -v a="$from" -v b="$to" '$1 >= a && $1 <= b' "$D/$f.log"
done
