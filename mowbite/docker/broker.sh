# shared by recorder.sh and scheduler.sh: where the mqtt broker is and how to log in
PORT=${MOWER_MQTT_PORT:-1883}
AUTH="${MOWER_MQTT_USER:+-u $MOWER_MQTT_USER} ${MOWER_MQTT_PASSWORD:+-P $MOWER_MQTT_PASSWORD}"

# the broker runs on the mower: localhost with host networking, otherwise the docker gateway
find_broker() {
  for h in $MOWER_MQTT_HOST 127.0.0.1 host.docker.internal $(ip route | awk '/^default/ {print $3; exit}'); do
    nc -z -w 2 "$h" "$PORT" 2>/dev/null && echo "$h" && return
  done
}
