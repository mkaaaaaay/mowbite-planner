#!/bin/sh
# map backups in the /data volume, gzipped. one <id>.json.gz per backup and <id>.meta with
# name, number of areas and whether it was made automatically before a save
# the app on another mower may ask too (several mowers in one app)
# after a Status line, busybox httpd only reads that when it comes first
cors() { printf 'Access-Control-Allow-Origin: *\r\n'; }
D=/data/backups
KEEP_AUTO=30
mkdir -p "$D"

param() { printf '%s' "$QUERY_STRING" | tr '&' '\n' | sed -n "s/^$1=//p" | head -n1; }
fail() { printf 'Status: %s\r\n' "$1"; cors; printf 'Content-Type: text/plain\r\n\r\n%s\n' "$2"; exit 0; }
json_str() { printf '%s' "$1" | tr -d '\000-\037' | sed 's/\\/\\\\/g; s/"/\\"/g'; }

id=$(param id)
case "$id" in
  '' | *[!0-9-]*) id= ;;
esac

if [ "$REQUEST_METHOD" = "POST" ]; then
  action=$(param action)
  if [ "$action" = "delete" ]; then
    [ -n "$id" ] || fail 400 'no id'
    rm -f "$D/$id.json.gz" "$D/$id.meta"
    cors
    printf 'Content-Type: application/json\r\n\r\n{"ok":true}\n'
    exit 0
  fi
  len=${CONTENT_LENGTH:-0}
  if [ "$len" -le 0 ] || [ "$len" -gt 2000000 ]; then fail 413 'too large'; fi
  new="$(date +%s)-$$"
  name=$(httpd -d "$(param name)" | tr -d '\000-\037' | cut -c1-60)
  areas=$(param areas | tr -cd '0-9')
  auto=$(param auto | tr -cd '01')
  head -c "$len" | gzip -9 > "$D/$new.json.gz"
  printf '%s\n%s\n%s\n' "$name" "${areas:-0}" "${auto:-0}" > "$D/$new.meta"
  # only the automatic ones are thinned out, the ones made by hand stay
  ls "$D"/*.meta 2>/dev/null | sort -r | while read -r m; do
    [ "$(sed -n 3p "$m")" = 1 ] && echo "$m"
  done | tail -n +$((KEEP_AUTO + 1)) | while read -r m; do
    rm -f "$m" "${m%.meta}.json.gz"
  done
  cors
  printf 'Content-Type: application/json\r\n\r\n{"id":"%s"}\n' "$new"
  exit 0
fi

if [ -n "$id" ]; then
  [ -f "$D/$id.json.gz" ] || fail 404 'not found'
  cors
  printf 'Content-Type: application/json\r\nCache-Control: no-store\r\n\r\n'
  zcat "$D/$id.json.gz"
  exit 0
fi

cors
printf 'Content-Type: application/json\r\nCache-Control: no-store\r\n\r\n['
first=1
for m in $(ls "$D"/*.meta 2>/dev/null | sort -r); do
  b=$(basename "$m" .meta)
  [ -f "$D/$b.json.gz" ] || continue
  [ $first = 1 ] || printf ','
  first=0
  printf '{"id":"%s","t":%s,"name":"%s","areas":%s,"auto":%s,"size":%s}' \
    "$b" "${b%%-*}" "$(json_str "$(sed -n 1p "$m")")" "$(sed -n 2p "$m" | tr -cd '0-9' | sed 's/^$/0/')" \
    "$([ "$(sed -n 3p "$m")" = 1 ] && echo true || echo false)" "$(wc -c < "$D/$b.json.gz")"
done
printf ']\n'
