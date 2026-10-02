#!/bin/sh
# memory, storage, load, temperature and uptime of the machine this container runs on, as "key value"
# lines. that's the mower when MowBite runs on it, "remote 1" says it runs elsewhere (a NAS or pc).
# for system.cgi and the 24 h history in recorder.sh
case "${MOWER_MQTT_HOST:-localhost}" in
  localhost | 127.0.0.1 | host.docker.internal) echo "remote 0" ;;
  *) echo "remote 1" ;;
esac
awk '/^MemTotal:/ { print "mem_total", $2 * 1024 } /^MemAvailable:/ { print "mem_available", $2 * 1024 }' /proc/meminfo
# the system disk (where docker keeps the images) and the data volume, in bytes
df -k / | awk 'NR == 2 { print "disk_total", $2 * 1024; print "disk_free", $4 * 1024 }'
df -k /data | awk 'NR == 2 { print "data_total", $2 * 1024; print "data_free", $4 * 1024 }'
awk '{ print "load", $1, $2, $3 }' /proc/loadavg
awk '{ print "uptime", int($1) }' /proc/uptime
grep -c ^processor /proc/cpuinfo | awk '{ print "cpus", $1 }'
# the cpu's zone (cpu-thermal on a raspberry), else the first one, in millidegrees
t=""
for z in /sys/class/thermal/thermal_zone*; do
  [ -r "$z/temp" ] || continue
  [ -n "$t" ] || t=$z/temp
  case "$(cat "$z/type" 2>/dev/null)" in *cpu* | *soc* | *CPU*) t=$z/temp; break ;; esac
done
[ -n "$t" ] && awk '{ print "cpu_temp", $1 / 1000 }' "$t"
