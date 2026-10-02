# input: "<unix time> <topic> <value>" from mosquitto_sub. output, appended to OUT:
#   charge <start> <end> <minutes> <volts at start> <volts at the end>   from charging on until the charger says Done
#   run <start> <end> <blade minutes> <volts after charging> <volts when charging starts again>   between two charges
# an older battery takes a shorter charge and loses more volts per hour of mowing. the state in between (a run since
# the last charge, a charge going on) is kept in STATE, so a restart of the container doesn't lose the run it was in
function save() {
  if (!STATE) return
  printf "%d %.2f %.1f %d %d %.2f\n", run_t, run_v, blade_s, charging, charge_t, charge_v > STATE
  close(STATE)
  saved = t
}
BEGIN {
  OUT = ENVIRON["OUT"]
  STATE = ENVIRON["STATE"]
  if (STATE && (getline line < STATE) > 0) {
    split(line, s, " ")
    run_t = s[1] + 0; run_v = s[2] + 0; blade_s = s[3] + 0; charging = s[4] + 0; charge_t = s[5] + 0; charge_v = s[6] + 0
  }
  if (STATE) close(STATE)
}

{
  t = $1 + 0
  id = $2
  sub(/.*sensors\//, "", id)
  sub(/\/data$/, "", id)
  # the rest after time and topic. index($0, $3) found a "0" in the time already, a 0 read as a big number
  val = $0
  sub(/^[^ ]+ [^ ]+ /, "", val)

  if (id == "om_v_battery") volts = val + 0
  else if (id == "om_mow_motor_rpm") {
    # blade time between two readings, gaps of the stream don't count
    if (blade && t - rpm_t < 15) blade_s += t - rpm_t
    blade = val + 0 > 500
    rpm_t = t
    # the blade time of the run, once a minute is enough
    if (run_t && t - saved > 60) save()
  } else if (id == "om_charge_current" && val + 0 > 0.1) {
    no_current_t = 0
    if (!charging && volts) {
      if (run_t) printf "run %d %d %.1f %.2f %.2f\n", run_t, t, blade_s / 60, run_v, volts >> OUT
      close(OUT)
      run_t = 0
      charging = 1; charge_t = t; charge_v = volts
      save()
    }
  } else if (id == "om_charge_current" && charging) {
    # off the charger before it said Done, e.g. a minute in the dock and off mowing again: after two minutes without
    # current the charge ends and the run starts from when the current stopped. it waited for a Done before and lost
    # the run
    if (!no_current_t) no_current_t = t
    else if (t - no_current_t > 120) {
      printf "charge %d %d %.1f %.2f %.2f\n", charge_t, no_current_t, (no_current_t - charge_t) / 60, charge_v, volts >> OUT
      close(OUT)
      charging = 0
      run_t = no_current_t; run_v = volts; blade_s = 0
      save()
    }
  } else if (id == "om_charge_state" && val == "Done" && charging) {
    printf "charge %d %d %.1f %.2f %.2f\n", charge_t, t, (t - charge_t) / 60, charge_v, volts >> OUT
    close(OUT)
    charging = 0
    run_t = t; run_v = volts; blade_s = 0
    save()
  }
}
