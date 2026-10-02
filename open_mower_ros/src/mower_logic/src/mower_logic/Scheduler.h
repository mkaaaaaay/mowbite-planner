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
#ifndef SRC_SCHEDULER_H
#define SRC_SCHEDULER_H

#include <ctime>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

// Starts mowing at set times, kept in schedule.json next to map.json so every app sees the same schedule.
// Apps read and change it through the schedule.* RPCs, what it does shows up as SCHEDULE_* events.
class Scheduler {
 public:
  struct Start {
    std::vector<int> days;  // 1 = Monday ... 7 = Sunday
    int time;               // minutes after midnight
    int end = -1;           // sent home then (minutes after midnight, the next day when not after time), -1: none
    std::vector<std::string> areas;  // mowing area ids, empty: all
    bool dark = false;               // may start and mow in the dark
    bool sunset = false;             // sent home at sunset
    bool fresh = false;              // an interrupted job is dropped first, so it begins with the first area
  };

  struct Schedule {
    bool enabled = false;
    std::string timezone;  // IANA name, e.g. "Europe/Berlin", empty: the system's
    int min_battery = 0;   // percent, waits up to two hours for it
    bool skip_rain = true;
    std::vector<Start> starts;
    std::string pause_until;                // YYYY-MM-DD, up to and including that day, empty: no pause
    std::vector<std::string> pause_areas;  // empty: all of them
  };

  static Scheduler INSTANCE;

  void load();
  // checks what's due and looks after a run it started, every few seconds
  void tick();

  nlohmann::json get();
  // validates and stores a schedule in the json format of get(), throws std::invalid_argument
  nlohmann::json set(const nlohmann::json& j);
  // the next start as {"t": unix time, "start": index}, null when nothing is coming up
  nlohmann::json next();

  static Schedule parse(const nlohmann::json& j);
  static nlohmann::json toJson(const Schedule& s);

 private:
  std::mutex mutex;
  Schedule schedule;

  // a start that's due, until it's started or skipped
  struct Pending {
    int index;
    Start start;
    time_t day_start;  // midnight of the day it was due
    time_t give_up;    // no longer waits for the battery after this
    bool waiting = false;
  };
  std::optional<Pending> pending;
  std::string last_due;  // "YYYY-MM-DD HH:MM" of the last start taken up, so it's not taken twice
  // a run it started, sent home at stop (unix time, 0: never)
  bool running = false;
  time_t stop = 0;
  time_t started_at = 0;
  std::string stop_reason;

  void applyTimezone();
  void save();
  void evaluate(time_t now);
  void skip(const std::string& reason);
  // today's sunrise and sunset in minutes after midnight from the gps datum, false without one
  bool sunTimes(time_t day_start, int& rise, int& set);
  bool isDark(time_t now);
};

#endif  // SRC_SCHEDULER_H
