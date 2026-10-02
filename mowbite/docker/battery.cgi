#!/bin/sh
# the charge and discharge cycles battery.sh kept, for the sensors page
printf 'Access-Control-Allow-Origin: *\r\n'
printf 'Content-Type: text/plain\r\nCache-Control: no-store\r\n\r\n'
cat /data/battery.log 2>/dev/null
