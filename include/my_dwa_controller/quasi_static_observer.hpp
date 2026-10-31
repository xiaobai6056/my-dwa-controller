#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <utility>
#include <vector>

namespace my_dwa_controller
{
using Point = std::array<double, 2>;

class QuasiStaticObserver
{
public:
  struct Result
  {
    bool permission;
    std::size_t frames;
    int64_t span_ns;
    double diameter;
    Point median;
    const char * reason;
  };

  // Points are the associated compact object's source-time odom coordinates.
  // The owner resets on event/identity changes; this component owns no identity.
  Result update(
    int64_t stamp, int64_t now, const std::vector<Point> & points,
    bool observation_valid, bool robot_stopped, bool moving);
  void reset();

private:
  std::deque<std::pair<int64_t, Point>> window_;
  bool have_last_ = false;
  int64_t last_stamp_ = 0;
};

class RobotStopObserver
{
public:
  void feed(int64_t stamp, double vx, double vy, double w);
  // Coverage only: wait for odom to reach Scan before consuming M evidence.
  bool covers(int64_t scan_stamp) const;
  bool stoppedAt(int64_t scan_stamp, int64_t now) const;
  void reset();

private:
  struct Snapshot
  {
    int64_t stamp;
    int64_t quiet_since;
    bool quiet;
  };
  mutable std::mutex mutex_;
  std::deque<Snapshot> history_;
  bool have_last_ = false, quiet_ = false;
  int64_t last_stamp_ = 0, quiet_since_ = 0;
};
}  // namespace my_dwa_controller
