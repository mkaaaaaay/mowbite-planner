// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//
#include "Scheduler.h"

#include <ros/ros.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <regex>
#include <stdexcept>

#include "behaviors/IdleBehavior.h"
#include "behaviors/MowingBehavior.h"
#include "mower_logic/MowerLogicConfig.h"
#include "mower_msgs/HighLevelStatus.h"
#include "mower_msgs/Status.h"

using njson = nlohmann::json;

extern Behavior* currentBehavior;
extern mower_msgs::HighLevelStatus high_level_status;
extern ros::Time rain_resume;
extern mower_msgs::Status getStatus();
extern mower_logic::MowerLogicConfig getConfig();
extern void setConfig(mower_logic::MowerLogicConfig);
extern void publishMowerEvent(const std::string& type, json details);
extern std::vector<std::string> mowingAreaIds();

Scheduler Scheduler::INSTANCE;

namespace {
const char* FILE_NAME = "schedule.json";
// how long a due start waits for the battery
const int BATTERY_WAIT = 2 * 3600;
// a run counts as done when the mower is idle without an open job, not right after the start though
const int START_GRACE = 120;

[[noreturn]] void invalid(const std::string& message) {
  throw std::invalid_argument(message);
}

int parseClock(const njson& v, const std::string& key) {
  int h, m;
  char rest;
  if (!v.is_string() || std::sscanf(v.get<std::string>().c_str(), "%2d:%2d%c", &h, &m, &rest) != 2 || h < 0 ||
      h > 23 || m < 0 || m > 59) {
    invalid("not a time like 07:30: " + key);
  }
  return h * 60 + m;
}

std::string clock(int minutes) {
  char buf[6];
  std::snprintf(buf, sizeof(buf), "%02d:%02d", minutes / 60, minutes % 60);
  return buf;
}

std::vector<std::string> parseIds(const njson& v, const std::string& key) {
  if (!v.is_array()) invalid("not a list of area ids: " + key);
  std::vector<std::string> ids;
  for (const auto& a : v) {
    if (!a.is_string() || a.get<std::string>().empty()) invalid("not a list of area ids: " + key);
    ids.push_back(a.get<std::string>());
  }
  return ids;
}

bool flag(const njson& j, const char* key, bool fallback) {
  if (!j.contains(key)) return fallback;
  if (!j[key].is_boolean()) invalid(std::string("not true or false: ") + key);
  return j[key].get<bool>();
}

// The garden's time zone, worked out here: setenv("TZ") and tzset() change the environment of the whole process,
// which other threads read meanwhile. From the zoneinfo file: its list of changes, and after the last one the POSIX
// rule at its end, e.g. "CET-1CEST,M3.5.0,M10.5.0/3". Offsets are in seconds east of UTC (POSIX writes them west).
// Only used under the scheduler's lock
struct Zone {
  struct Rule {
    char kind = 'M';  // M: month.week.day, J: day 1-365 without Feb 29, n: day 0-365
    int month = 0, week = 0, day = 0;
    long time = 7200;  // local time of the change, seconds after midnight
  };
  long std_offset = 0, dst_offset = 0;
  bool has_dst = false;
  Rule start, end;
  // the listed changes (unix time) with the offset from then on, the rule only counts after the last one. some
  // zones have more listed than the rule tells, e.g. Morocco's summer time paused every Ramadan
  std::vector<int64_t> changes;
  std::vector<long> offsets;
};
Zone zone;

long number(const char*& p) {
  char* e;
  const long v = std::strtol(p, &e, 10);
  p = e;
  return v;
}

bool zoneName(const char*& p) {
  if (*p == '<') {
    while (*p && *p != '>') p++;
    if (*p != '>') return false;
    p++;
    return true;
  }
  const char* from = p;
  while (std::isalpha(static_cast<unsigned char>(*p))) p++;
  return p - from >= 3;
}

// [+-]hh[:mm[:ss]] in seconds
bool zoneTime(const char*& p, long& out) {
  int sign = 1;
  if (*p == '+' || *p == '-') sign = *p++ == '-' ? -1 : 1;
  if (!std::isdigit(static_cast<unsigned char>(*p))) return false;
  long t = number(p) * 3600;
  if (*p == ':') {
    p++;
    t += number(p) * 60;
    if (*p == ':') {
      p++;
      t += number(p);
    }
  }
  out = sign * t;
  return true;
}

bool zoneRule(const char*& p, Zone::Rule& r) {
  if (*p == 'M') {
    p++;
    r.kind = 'M';
    r.month = number(p);
    if (*p++ != '.') return false;
    r.week = number(p);
    if (*p++ != '.') return false;
    r.day = number(p);
    if (r.month < 1 || r.month > 12 || r.week < 1 || r.week > 5 || r.day < 0 || r.day > 6) return false;
  } else if (*p == 'J') {
    p++;
    r.kind = 'J';
    r.day = number(p);
  } else if (std::isdigit(static_cast<unsigned char>(*p))) {
    r.kind = 'n';
    r.day = number(p);
  } else {
    return false;
  }
  r.time = 7200;
  if (*p == '/') {
    p++;
    if (!zoneTime(p, r.time)) return false;
  }
  return true;
}

bool parseZone(const std::string& rule, Zone& z) {
  const char* p = rule.c_str();
  long off;
  if (!zoneName(p) || !zoneTime(p, off)) return false;
  z = Zone();
  z.std_offset = -off;
  if (!*p) return true;
  if (!zoneName(p)) return false;
  z.has_dst = true;
  z.dst_offset = z.std_offset + 3600;
  if (*p && *p != ',') {
    if (!zoneTime(p, off)) return false;
    z.dst_offset = -off;
  }
  if (*p++ != ',' || !zoneRule(p, z.start) || *p++ != ',' || !zoneRule(p, z.end)) return false;
  return *p == 0;
}

// the zone of a name like "Europe/Berlin", empty: the system's (UTC when it has none). false when there's no such
// zone or its file isn't a zoneinfo file of version 2 or later
bool loadZone(const std::string& name, Zone& z) {
  std::ifstream f(name.empty() ? "/etc/localtime" : "/usr/share/zoneinfo/" + name, std::ios::binary);
  if (!f) {
    z = Zone();
    return name.empty();
  }
  const std::string data((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  const auto byte = [&](size_t at) { return static_cast<unsigned char>(data[at]); };
  const auto be = [&](size_t at, int bytes) {
    uint64_t v = 0;
    for (int i = 0; i < bytes; i++) v = v << 8 | byte(at + i);
    return v;
  };
  // header: magic, version, 15 reserved, the counts isut, isstd, leap, time, type, char
  const auto counts = [&](size_t at, uint64_t c[6]) {
    if (data.size() < at + 44 || data.compare(at, 4, "TZif") != 0) return false;
    for (int i = 0; i < 6; i++) c[i] = be(at + 20 + 4 * i, 4);
    return true;
  };
  uint64_t c[6];
  if (!counts(0, c) || data.size() < 5 || byte(4) < '2') return false;
  // the 32 bit part comes first, the 64 bit one after it
  const size_t second = 44 + c[3] * 5 + c[4] * 6 + c[5] + c[2] * 8 + c[1] + c[0];
  if (!counts(second, c)) return false;
  const uint64_t isut = c[0], isstd = c[1], leap = c[2], times = c[3], types = c[4], chars = c[5];
  size_t at = second + 44;
  const size_t footer = at + times * 9 + types * 6 + chars + leap * 12 + isstd + isut;
  if (data.size() < footer + 2 || data[footer] != '\n' || types == 0) return false;
  const size_t footer_end = data.find('\n', footer + 1);
  if (footer_end == std::string::npos) return false;
  Zone result;
  const std::string rule = data.substr(footer + 1, footer_end - footer - 1);
  if (!rule.empty() && !parseZone(rule, result)) return false;
  const size_t index_at = at + times * 8, types_at = index_at + times;
  const auto type_offset = [&](uint64_t type) { return static_cast<long>(static_cast<int32_t>(be(types_at + 6 * type, 4))); };
  for (uint64_t i = 0; i < times; i++) {
    const uint64_t type = byte(index_at + i);
    if (type >= types) return false;
    result.changes.push_back(static_cast<int64_t>(be(at + 8 * i, 8)));
    result.offsets.push_back(type_offset(type));
  }
  // without a rule the last listed offset goes on
  if (rule.empty()) result.std_offset = result.offsets.empty() ? type_offset(0) : result.offsets.back();
  // before the first change: the first type
  result.offsets.insert(result.offsets.begin(), type_offset(0));
  z = result;
  return true;
}

// days since 1970-01-01 of a date (proleptic gregorian)
long daysFromCivil(long y, int m, int d) {
  y -= m <= 2;
  const long era = (y >= 0 ? y : y - 399) / 400;
  const long yoe = y - era * 400;
  const long doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

bool leapYear(long y) {
  return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
}

// the day a rule falls on in that year, days since 1970-01-01
long ruleDay(long y, const Zone::Rule& r) {
  if (r.kind == 'J') return daysFromCivil(y, 1, 1) + r.day - 1 + (leapYear(y) && r.day >= 60 ? 1 : 0);
  if (r.kind == 'n') return daysFromCivil(y, 1, 1) + r.day;
  const long first = daysFromCivil(y, r.month, 1);
  // 1970-01-01 was a thursday
  const long first_weekday = ((first + 4) % 7 + 7) % 7;
  long d = first + (r.day - first_weekday + 7) % 7 + (r.week - 1) * 7;
  const long next_month = r.month == 12 ? daysFromCivil(y + 1, 1, 1) : daysFromCivil(y, r.month + 1, 1);
  while (d >= next_month) d -= 7;
  return d;
}

// seconds east of UTC at that moment
long utcOffset(time_t t) {
  if (!zone.changes.empty() && t < zone.changes.back()) {
    const auto after = std::upper_bound(zone.changes.begin(), zone.changes.end(), static_cast<int64_t>(t));
    return zone.offsets[after - zone.changes.begin()];
  }
  if (!zone.has_dst) return zone.std_offset;
  struct tm tm;
  gmtime_r(&t, &tm);
  const long y = tm.tm_year + 1900;
  // the change to summer time is given in standard time, the one back in summer time
  const long start = ruleDay(y, zone.start) * 86400 + zone.start.time - zone.std_offset;
  const long end = ruleDay(y, zone.end) * 86400 + zone.end.time - zone.dst_offset;
  const bool dst = start < end ? t >= start && t < end : t >= start || t < end;
  return dst ? zone.dst_offset : zone.std_offset;
}

// the local date and time of t, the fields of struct tm
struct tm local(time_t t) {
  const time_t shifted = t + utcOffset(t);
  struct tm tm;
  gmtime_r(&shifted, &tm);
  return tm;
}

// local midnight of the day t lies in, the time `minutes` after it (also across a dst change)
time_t localTime(time_t day, int minutes) {
  const struct tm d = local(day);
  const time_t wall = daysFromCivil(d.tm_year + 1900, d.tm_mon + 1, d.tm_mday) * 86400 + minutes * 60;
  // the offset of the moment it'll be, found from the one before
  const time_t guess = wall - utcOffset(wall - zone.std_offset);
  return wall - utcOffset(guess);
}

std::string date(time_t t) {
  const struct tm tm = local(t);
  char buf[11];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d", &tm);
  return buf;
}

int minutesOfDay(time_t t) {
  const struct tm tm = local(t);
  return tm.tm_hour * 60 + tm.tm_min;
}

int weekday(time_t t) {
  const struct tm tm = local(t);
  return tm.tm_wday == 0 ? 7 : tm.tm_wday;
}

bool isIdle() {
  Behavior* b = currentBehavior;
  return b == &IdleBehavior::INSTANCE || b == &IdleBehavior::DOCKED_INSTANCE;
}
}  // namespace

Scheduler::Schedule Scheduler::parse(const njson& j) {
  if (!j.is_object()) invalid("the schedule must be an object");
  Schedule s;
  s.enabled = flag(j, "enabled", false);
  s.skip_rain = flag(j, "skip_rain", true);
  if (j.contains("timezone")) {
    if (!j["timezone"].is_string()) invalid("not a time zone name: timezone");
    s.timezone = j["timezone"].get<std::string>();
    static const std::regex name("[A-Za-z0-9_+-]+(/[A-Za-z0-9_+-]+)*");
    Zone z;
    if (!s.timezone.empty() && (!std::regex_match(s.timezone, name) || !loadZone(s.timezone, z))) {
      invalid("unknown time zone: " + s.timezone);
    }
  }
  if (j.contains("min_battery")) {
    if (!j["min_battery"].is_number_integer() || j["min_battery"].get<double>() < 0 ||
        j["min_battery"].get<double>() > 100) {
      invalid("not a percentage from 0 to 100: min_battery");
    }
    s.min_battery = j["min_battery"].get<int>();
  }
  if (j.contains("starts")) {
    if (!j["starts"].is_array()) invalid("not a list: starts");
    for (const auto& o : j["starts"]) {
      if (!o.is_object()) invalid("a start must be an object");
      Start start;
      if (!o.contains("days") || !o["days"].is_array() || o["days"].empty()) invalid("a start needs days (1-7)");
      for (const auto& d : o["days"]) {
        if (!d.is_number_integer() || d.get<double>() < 1 || d.get<double>() > 7) invalid("days go from 1 to 7");
        if (std::find(start.days.begin(), start.days.end(), d.get<int>()) == start.days.end()) {
          start.days.push_back(d.get<int>());
        }
      }
      std::sort(start.days.begin(), start.days.end());
      if (!o.contains("time")) invalid("a start needs a time");
      start.time = parseClock(o["time"], "time");
      if (o.contains("end") && !o["end"].is_null()) start.end = parseClock(o["end"], "end");
      if (o.contains("areas")) start.areas = parseIds(o["areas"], "areas");
      start.dark = flag(o, "dark", false);
      start.sunset = flag(o, "sunset", false);
      start.fresh = flag(o, "fresh", false);
      s.starts.push_back(start);
    }
  }
  if (j.contains("pause") && !j["pause"].is_null()) {
    const auto& p = j["pause"];
    static const std::regex day("[0-9]{4}-[0-9]{2}-[0-9]{2}");
    if (!p.is_object() || !p.contains("until") || !p["until"].is_string() ||
        !std::regex_match(p["until"].get<std::string>(), day)) {
      invalid("a pause needs until as YYYY-MM-DD");
    }
    s.pause_until = p["until"].get<std::string>();
    if (p.contains("areas")) s.pause_areas = parseIds(p["areas"], "pause.areas");
  }
  return s;
}

njson Scheduler::toJson(const Schedule& s) {
  njson starts = njson::array();
  for (const auto& st : s.starts) {
    njson o = {{"days", st.days}, {"time", clock(st.time)}};
    if (st.end >= 0) o["end"] = clock(st.end);
    if (!st.areas.empty()) o["areas"] = st.areas;
    if (st.dark) o["dark"] = true;
    if (st.sunset) o["sunset"] = true;
    if (st.fresh) o["fresh"] = true;
    starts.push_back(o);
  }
  njson j = {{"enabled", s.enabled},
            {"timezone", s.timezone},
            {"min_battery", s.min_battery},
            {"skip_rain", s.skip_rain},
            {"starts", starts}};
  if (!s.pause_until.empty()) {
    j["pause"] = {{"until", s.pause_until}};
    if (!s.pause_areas.empty()) j["pause"]["areas"] = s.pause_areas;
  }
  return j;
}

void Scheduler::load() {
  std::lock_guard<std::mutex> lk{mutex};
  std::ifstream f(FILE_NAME);
  if (f.good()) {
    try {
      schedule = parse(njson::parse(f));
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Scheduler: can't read " << FILE_NAME << ", no schedule: " << e.what());
      schedule = Schedule();
    }
  }
  applyTimezone();
}

void Scheduler::save() {
  const std::string tmp = std::string(FILE_NAME) + ".tmp";
  {
    std::ofstream f(tmp);
    f << toJson(schedule).dump(2);
    if (!f.good()) throw std::runtime_error("can't write the schedule");
  }
  std::filesystem::rename(tmp, FILE_NAME);
}

// the time zone of the garden for the local times here (under the lock)
void Scheduler::applyTimezone() {
  if (!loadZone(schedule.timezone, zone)) {
    ROS_ERROR_STREAM("Scheduler: can't read the time zone " << schedule.timezone << ", using UTC");
    zone = Zone();
  }
}

njson Scheduler::get() {
  std::lock_guard<std::mutex> lk{mutex};
  return toJson(schedule);
}

njson Scheduler::set(const njson& j) {
  Schedule s = parse(j);
  std::lock_guard<std::mutex> lk{mutex};
  schedule = s;
  save();
  applyTimezone();
  // a start that's waiting keeps its settings, it just won't come again once taken off
  return toJson(schedule);
}

njson Scheduler::next() {
  std::lock_guard<std::mutex> lk{mutex};
  if (!schedule.enabled) return nullptr;
  const time_t now = time(nullptr);
  for (int d = 0; d < 8; d++) {
    const time_t day = localTime(now + d * 86400, 12 * 60);
    const bool paused = !schedule.pause_until.empty() && schedule.pause_areas.empty() && date(day) <= schedule.pause_until;
    if (paused) continue;
    time_t best = 0;
    int index = -1;
    for (size_t i = 0; i < schedule.starts.size(); i++) {
      const auto& st = schedule.starts[i];
      if (std::find(st.days.begin(), st.days.end(), weekday(day)) == st.days.end()) continue;
      const time_t t = localTime(day, st.time);
      if (t <= now || (best && t >= best)) continue;
      // one in the dark without being allowed there is left out when it's due, so it isn't the next one
      if (!st.dark && isDark(t)) continue;
      best = t;
      index = i;
    }
    if (index >= 0) return {{"t", best}, {"start", index}};
  }
  return nullptr;
}

bool Scheduler::sunTimes(time_t day_start, int& rise, int& set) {
  // where the gps driver takes it from (both platforms), the ros 1 gps driver of older versions as a fallback
  double lat, lon;
  if (!(ros::param::get("/ll/services/gps/datum_lat", lat) && ros::param::get("/ll/services/gps/datum_long", lon)) &&
      !(ros::param::get("/xbot_driver_gps/datum_lat", lat) && ros::param::get("/xbot_driver_gps/datum_long", lon))) {
    return false;
  }
  // NOAA solar calculator, at noon (UTC) of the local date
  const time_t noon_local = localTime(day_start, 12 * 60);
  const long gmtoff = utcOffset(noon_local);
  const double offset = gmtoff / 60.0;
  const double noon = noon_local + gmtoff;
  const auto rad = [](double d) { return d * M_PI / 180; };
  const auto deg = [](double r) { return r * 180 / M_PI; };
  const double t = (noon / 86400 + 2440587.5 - 2451545) / 36525;
  const double L = std::fmod(280.46646 + t * (36000.76983 + t * 0.0003032), 360);
  const double M = 357.52911 + t * (35999.05029 - 0.0001537 * t);
  const double e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
  const double C = std::sin(rad(M)) * (1.914602 - t * (0.004817 + 0.000014 * t)) +
                   std::sin(rad(2 * M)) * (0.019993 - 0.000101 * t) + std::sin(rad(3 * M)) * 0.000289;
  const double om = 125.04 - 1934.136 * t;
  const double lam = L + C - 0.00569 - 0.00478 * std::sin(rad(om));
  const double eps =
      23 + (26 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60) / 60 + 0.00256 * std::cos(rad(om));
  const double decl = deg(std::asin(std::sin(rad(eps)) * std::sin(rad(lam))));
  const double y = std::pow(std::tan(rad(eps / 2)), 2);
  const double eq = 4 * deg(y * std::sin(2 * rad(L)) - 2 * e * std::sin(rad(M)) +
                            4 * e * y * std::sin(rad(M)) * std::cos(2 * rad(L)) - 0.5 * y * y * std::sin(4 * rad(L)) -
                            1.25 * e * e * std::sin(2 * rad(M)));
  const double c = (std::cos(rad(90.833)) - std::sin(rad(lat)) * std::sin(rad(decl))) /
                   (std::cos(rad(lat)) * std::cos(rad(decl)));
  // polar day or night
  if (c < -1 || c > 1) return false;
  const double ha = deg(std::acos(c));
  const double n = 720 - 4 * lon - eq + offset;
  rise = std::clamp(static_cast<int>(std::lround(n - 4 * ha)), 0, 1439);
  set = std::clamp(static_cast<int>(std::lround(n + 4 * ha)), 0, 1439);
  return true;
}

// between sunset and sunrise, or 6 pm to 6 am without a gps datum
bool Scheduler::isDark(time_t now) {
  int rise = 6 * 60, set = 18 * 60;
  sunTimes(now, rise, set);
  const int m = minutesOfDay(now);
  return m < rise || m >= set;
}

void Scheduler::skip(const std::string& reason) {
  ROS_INFO_STREAM("Scheduler: start " << pending->index << " skipped: " << reason);
  publishMowerEvent("SCHEDULE_SKIP", json{{"start", pending->index}, {"reason", reason}});
  pending.reset();
}

void Scheduler::evaluate(time_t now) {
  auto& p = *pending;
  const Start& st = p.start;
  const bool paused_today = !schedule.pause_until.empty() && date(now) <= schedule.pause_until;
  if (paused_today && schedule.pause_areas.empty()) return skip("paused");

  // the areas the run mows: the picked ones (all when none) that are still in the map, less the paused ones
  std::vector<std::string> areas;
  if (!st.areas.empty() || paused_today) {
    for (const auto& id : mowingAreaIds()) {
      const bool picked = st.areas.empty() || std::count(st.areas.begin(), st.areas.end(), id);
      const bool off = paused_today && std::count(schedule.pause_areas.begin(), schedule.pause_areas.end(), id);
      if (picked && !off) areas.push_back(id);
    }
    if (areas.empty()) return skip(paused_today ? "paused" : "no_areas");
    // paused ones not being among the picked ones changes nothing
    if (st.areas.empty() && areas.size() == mowingAreaIds().size()) areas.clear();
  }

  // the end of this start: after its time on the same day, otherwise the next day
  time_t end = 0;
  if (st.end >= 0) {
    end = localTime(p.day_start, st.end);
    if (st.end <= st.time) end = localTime(p.day_start + 36 * 3600, st.end);
    if (now >= end) return skip("end_passed");
  }

  if (high_level_status.emergency) return skip("emergency");
  if (schedule.skip_rain && (getStatus().rain_detected || ros::Time::now() < rain_resume)) return skip("rain");
  if (!st.dark && isDark(now)) return skip("dark");
  if (!isIdle()) return skip("busy");
  const int battery = std::lround(high_level_status.battery_percent * 100);
  if (battery < schedule.min_battery) {
    if (now > p.give_up) return skip("battery");
    if (!p.waiting) {
      publishMowerEvent("SCHEDULE_WAIT", json{{"start", p.index}, {"reason", "battery"}, {"battery", battery}});
      p.waiting = true;
    }
    return;
  }

  if (st.fresh && MowingBehavior::INSTANCE.has_unfinished_job()) {
    currentBehavior->handle_action("mower_logic:idle/reset_job");
  }
  MowingBehavior::INSTANCE.set_job_areas(areas);
  currentBehavior->command_start();
  ROS_INFO_STREAM("Scheduler: started start " << p.index);
  json details = {{"start", p.index}, {"battery", battery}};
  if (!areas.empty()) details["areas"] = areas;
  publishMowerEvent("SCHEDULE_START", details);

  // home at the end time, and at sunset when wanted or when the end lies in the dark without mowing in the dark
  running = true;
  stop = end;
  stop_reason = "end";
  if (st.sunset || (end && !st.dark && isDark(end))) {
    int rise = 6 * 60, set = 18 * 60;
    sunTimes(now, rise, set);
    const time_t dusk = localTime(p.day_start, set);
    if (dusk > now && (!stop || dusk < stop)) {
      stop = dusk;
      stop_reason = "sunset";
    }
  }
  started_at = now;
  pending.reset();
}

void Scheduler::tick() {
  std::lock_guard<std::mutex> lk{mutex};
  const time_t now = time(nullptr);

  // a run it started: done once the mower is idle without an open job, sent home at the stop time. the job
  // stays open then and is paused by hand, so it doesn't carry on by itself after charging
  if (running && now - started_at > START_GRACE) {
    if (isIdle() && !MowingBehavior::INSTANCE.has_unfinished_job()) {
      running = false;
    } else if (stop && now >= stop) {
      auto config = getConfig();
      config.manual_pause_mowing = true;
      setConfig(config);
      if (!isIdle()) currentBehavior->command_home();
      ROS_INFO_STREAM("Scheduler: sent home, " << stop_reason);
      publishMowerEvent("SCHEDULE_STOP", json{{"reason", stop_reason}});
      running = false;
    }
  }

  if (schedule.enabled) {
    const int minute = minutesOfDay(now);
    const std::string key = date(now) + " " + clock(minute);
    if (key != last_due) {
      for (size_t i = 0; i < schedule.starts.size(); i++) {
        const auto& st = schedule.starts[i];
        if (st.time != minute || !std::count(st.days.begin(), st.days.end(), weekday(now))) continue;
        last_due = key;
        pending = Pending{static_cast<int>(i), st, localTime(now, 0), now + BATTERY_WAIT};
        break;
      }
    }
  } else {
    pending.reset();
  }

  if (pending) evaluate(now);
}
