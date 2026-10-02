#!/bin/sh
# keeps the last 24 h of sensor values (5 min buckets) in /data, so the sensors page has more
# than the time a browser had it open
. /broker.sh
. /openmower.sh

while :; do
  host=$(find_broker)
  if [ -n "$host" ]; then
    # shellcheck disable=SC2086
    # the machine's own values (system.sh) go in as sensors too, once a minute
    {
      while :; do
        /system.sh | awk -v t="$(date +%s)" '
          $1 == "mem_total" { mt = $2 } $1 == "mem_available" { ma = $2 } $1 == "disk_free" { print t, "sensors/sys_disk_free/data", $2 / 1e9 }
          $1 == "cpu_temp" { print t, "sensors/sys_cpu_temp/data", $2 } $1 == "load" { print t, "sensors/sys_load/data", $2 }
          END { if (mt) print t, "sensors/sys_mem/data", (mt - ma) / mt * 100 }'
        sleep 60
      done &
      sampler=$!
      mosquitto_sub -h "$host" -p "$PORT" $AUTH -t "${MOWER_MQTT_PREFIX}$TOPIC_SENSOR_DATA" -F '%U %t %p'
      kill "$sampler" 2>/dev/null
    } | OUT=/data/sensors.tsv awk -f /recorder.awk
  fi
  sleep 30
done
