# input: "<unix time> <topic> <value>" from mosquitto_sub
# file:  sensor, bucket start, min, max, last value. one line per sensor and 5 minutes
BEGIN {
  B = 300
  KEEP = 86400
  F = ENVIRON["OUT"]
  while ((getline line < F) > 0) {
    split(line, a, "\t")
    k = a[1] SUBSEP a[2]
    lo[k] = a[3]; hi[k] = a[4]; last[k] = a[5]
  }
  close(F)
}

{
  t = int($1)
  v = $3
  if (v !~ /^-?[0-9]+(\.[0-9]*)?([eE][-+]?[0-9]+)?$/) next
  id = $2
  sub(/.*sensors\//, "", id)
  sub(/\/data$/, "", id)
  v += 0
  # 999 is "no fix", not an accuracy
  if (id == "om_gps_accuracy" && v >= 999) next

  k = id SUBSEP (t - t % B)
  if (!(k in lo) || v < lo[k]) lo[k] = v
  if (!(k in hi) || v > hi[k]) hi[k] = v
  last[k] = v

  if (t - written >= 60) {
    save(t)
    written = t
  }
}

function save(now,    k, p, tmp) {
  tmp = F ".tmp"
  printf "" > tmp
  for (k in lo) {
    split(k, p, SUBSEP)
    if (p[2] + 0 < now - KEEP) {
      delete lo[k]; delete hi[k]; delete last[k]
      continue
    }
    printf "%s\t%s\t%s\t%s\t%s\n", p[1], p[2], lo[k], hi[k], last[k] >> tmp
  }
  close(tmp)
  system("mv " tmp " " F)
}
