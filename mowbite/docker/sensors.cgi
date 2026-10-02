#!/bin/sh
# last 24 h of sensor values written by recorder.sh
# the app on another mower may ask too (several mowers in one app)
printf 'Access-Control-Allow-Origin: *\r\n'
printf 'Content-Type: text/plain\r\nCache-Control: no-store\r\n\r\n'
cat /data/sensors.tsv 2>/dev/null
