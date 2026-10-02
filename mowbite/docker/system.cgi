#!/bin/sh
# what system.sh finds out, for the sensors page
printf 'Access-Control-Allow-Origin: *\r\n'
printf 'Content-Type: text/plain\r\nCache-Control: no-store\r\n\r\n'
/system.sh
