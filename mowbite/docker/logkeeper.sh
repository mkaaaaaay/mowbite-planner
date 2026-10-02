#!/bin/sh
# OpenMower keeps the last ros warnings and errors in memory only (logs.recent), a restart wipes them.
# every minute the new ones are fetched and kept in /data/roslog, one file per day (utc), 14 days, so the
# activity page has them for older problems too. lines: "<unix time> <LEVEL> <node>: <message>"
. /broker.sh
. /openmower.sh
D=/data/roslog
mkdir -p "$D"
TMP=/tmp/logkeeper.out

while :; do
  host=$(find_broker)
  if [ -n "$host" ]; then
    last=$(cat "$D/.last" 2>/dev/null)
    id="mowbite-logs-$(date +%s)"
    # the answer comes on the shared response topic, listen first and pick ours by its id
    # shellcheck disable=SC2086
    mosquitto_sub -h "$host" -p "$PORT" $AUTH -t "${MOWER_MQTT_PREFIX}$TOPIC_RPC_RESPONSE" -t "${MOWER_MQTT_PREFIX}$TOPIC_RPC_ERROR" -W 20 > "$TMP" 2>/dev/null &
    sub=$!
    sleep 1
    # shellcheck disable=SC2086
    mosquitto_pub -h "$host" -p "$PORT" $AUTH -t "${MOWER_MQTT_PREFIX}$TOPIC_RPC_REQUEST" \
      -m "{\"jsonrpc\":\"2.0\",\"method\":\"$RPC_LOGS\",\"params\":{\"since\":${last:-0},\"limit\":1000},\"id\":\"$id\"}" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      grep -q "\"$id\"" "$TMP" 2>/dev/null && break
      sleep 1
    done
    kill "$sub" 2>/dev/null
    wait "$sub" 2>/dev/null
    # the answer is pretty printed, keys sorted: an entry is level, msg, node, t on lines of their own, the id
    # comes last. others may answer something else at the same time, only the message with our id is kept
    new=$(awk -v id="\"$id\"" -v dir="$D" '
      function val(s) {
        sub(/^[^"]*"[a-z]+": /, "", s); sub(/,$/, "", s)
        if (s ~ /^".*"$/) { s = substr(s, 2, length(s) - 2); gsub(/\\"/, "\"", s); gsub(/\\n|\\t/, " ", s); gsub(/\\\\/, "\\", s) }
        return s
      }
      /^\{/ { n = 0; ours = 0 }
      /^    \{/ { lvl = msg = node = t = "" }
      /^      "level": / { lvl = val($0) }
      /^      "msg": / { msg = val($0) }
      /^      "node": / { node = val($0) }
      /^      "t": / { t = val($0) }
      /^    \}/ && t != "" { n++; ts[n] = t; line[n] = t " " lvl " " node ": " msg }
      index($0, "\"id\": " id) { ours = 1 }
      /^\}/ && ours {
        for (i = 1; i <= n; i++) {
          print line[i] >> (dir "/" strftime("%Y%m%d", int(ts[i])) ".log")
          if (ts[i] + 0 > max + 0) max = ts[i]
        }
      }
      END { if (max != "") print max }' "$TMP")
    [ -n "$new" ] && echo "$new" > "$D/.last"
    rm -f "$TMP"
    # 14 days
    ls -1 "$D"/*.log 2>/dev/null | sort -r | tail -n +15 | while read -r f; do rm -f "$f"; done
  fi
  sleep 60
done
