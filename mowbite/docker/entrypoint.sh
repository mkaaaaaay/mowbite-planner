#!/bin/sh
set -e

esc() { printf '%s' "$1" | sed 's/\\/\\\\/g; s/"/\\"/g'; }

printf 'window.__MOWER_CONFIG__ = {mqttUrl: "%s", mqttPrefix: "%s"};\n' \
  "$(esc "$MOWER_MQTT_WS_URL")" "$(esc "$MOWER_MQTT_PREFIX")" > /www/config.js

/recorder.sh &
/logkeeper.sh &
/battery.sh &
# a second copy (e.g. a dev container next to the one on the mower) shouldn't start the mower too
[ "$MOWBITE_SCHEDULER" = off ] || /scheduler.sh &

exec httpd -f -v -p "${PORT:-8080}" -h /www
