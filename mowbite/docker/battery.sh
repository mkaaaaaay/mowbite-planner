#!/bin/sh
# the battery over weeks: a line per charge and per discharge in /data/battery.log (battery.awk), about a year
. /broker.sh
. /openmower.sh
F=/data/battery.log
while :; do
  [ -f "$F" ] && tail -n 2000 "$F" > "$F.tmp" && mv "$F.tmp" "$F"
  host=$(find_broker)
  if [ -n "$host" ]; then
    # shellcheck disable=SC2086
    mosquitto_sub -h "$host" -p "$PORT" $AUTH -F '%U %t %p' \
      -t "${MOWER_MQTT_PREFIX}sensors/om_v_battery/data" -t "${MOWER_MQTT_PREFIX}sensors/om_charge_current/data" \
      -t "${MOWER_MQTT_PREFIX}sensors/om_charge_state/data" -t "${MOWER_MQTT_PREFIX}sensors/om_mow_motor_rpm/data" |
      OUT="$F" STATE=/data/battery.state awk -f /battery.awk
  fi
  sleep 30
done
