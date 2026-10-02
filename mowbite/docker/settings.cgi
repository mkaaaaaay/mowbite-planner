#!/bin/sh
# app settings shared by every device, kept in the /data volume
# the phone app reads and writes them too. X-Settings-Version is a checksum of what's stored: a save that says
# which version it started from (?v=...) gets a 409 with the current settings when another device saved in
# between, the app then puts its own change on top of those. a save without it just replaces them
# the app on another mower may ask too, after a Status line: busybox httpd only reads that when it comes first
cors() { printf 'Access-Control-Allow-Origin: *\r\nAccess-Control-Expose-Headers: X-Settings-Version\r\n'; }
F=/data/settings.json
# 0 while nothing is stored yet, so two devices starting from nothing don't overwrite each other either
version() { if [ -f "$F" ]; then cksum < "$F" | cut -d' ' -f1,2 | tr ' ' -; else echo 0; fi; }
# first and last character that isn't white space
ends() { tr -d ' \t\r\n' < "$1" | sed -n '1{s/^\(.\).*/\1/p;}'; tr -d ' \t\r\n' < "$1" | tail -c 1; }

if [ "$REQUEST_METHOD" = "POST" ]; then
  len=${CONTENT_LENGTH:-0}
  if [ "$len" -le 0 ] || [ "$len" -gt 65536 ]; then
    printf 'Status: 413 Too Large\r\n'
    cors
    printf 'Content-Type: text/plain\r\n\r\ntoo large\n'
    exit 0
  fi
  head -c "$len" > "$F.tmp"
  # only a json object, anything else would leave every device without settings
  if [ "$(ends "$F.tmp" | tr -d '\n')" != "{}" ]; then
    rm -f "$F.tmp"
    printf 'Status: 400 Bad Request\r\n'
    cors
    printf 'Content-Type: text/plain\r\n\r\nnot a json object\n'
    exit 0
  fi
  want=$(printf '%s' "$QUERY_STRING" | sed -n 's/^.*v=\([0-9-]*\).*$/\1/p')
  if [ -n "$want" ] && [ -f "$F" ] && [ "$want" != "$(version)" ]; then
    rm -f "$F.tmp"
    printf 'Status: 409 Conflict\r\n'
    cors
    printf 'X-Settings-Version: %s\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n' "$(version)"
    cat "$F"
    exit 0
  fi
  mv "$F.tmp" "$F"
fi

cors
printf 'X-Settings-Version: %s\r\nContent-Type: application/json\r\nCache-Control: no-store\r\n\r\n' "$(version)"
cat "$F" 2>/dev/null || printf '{}'
