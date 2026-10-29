#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include "my_dwa_controller/scan_obstacle_tracker.hpp"
#include "my_dwa_controller/types.hpp"
using namespace my_dwa_controller;
inline Pose2D point(double x, double y)
{
  Pose2D p;p.x = x;p.y = y;return p;
}

// Offline proposal only. Infinite forward extension is stricter than extending
// to a local-map edge and cannot create a front-tip route beyond that edge.
struct Exclusion
{
  double tx = 1, ty = 0, sign = 1, lo = 0, hi = 0, rear = 0, front = 0;
  int index = -1;
  int64_t stamp = 0;
  bool valid = false;
  double along(const Pose2D & p) const {return tx * p.x + ty * p.y;}
  double across(const Pose2D & p) const {return sign * (-ty * p.x + tx * p.y);}
  void envelope(const ObstacleObservation & o)
  {
    lo = rear = std::numeric_limits<double>::infinity();hi = front = -lo;
    for (const auto & p : o.points) {
      const auto q = point(p[0], p[1]);
      lo = std::min(lo, along(q));hi = std::max(hi, along(q));
      rear = std::min(rear, across(q));
      front = std::max(front, across(q));
    }
    lo -= .27;hi += .27;rear -= .27;front += .27;
    valid = !o.points.empty();
  }
  void start(const ScanSnapshot & s, int i, double x, double y)
  {
    tx = x;ty = y;index = i;
    const auto & o = s.obstacles.at(i);
    sign = (-ty * o.partial_motion_vx + tx * o.partial_motion_vy) > 0 ? 1 : -1;
    stamp = int64_t(s.stamp.sec) * 1000000000 + s.stamp.nanosec;
    envelope(o);
  }
  void observe(const ScanSnapshot & s, int64_t now)
  {
    const int64_t next = int64_t(s.stamp.sec) * 1000000000 + s.stamp.nanosec;
    if (next <= 0 || now < next || now - next > 350000000 || next < stamp) {
      valid = false;return;
    }
    if (next == stamp) {return;}
    int found = -1;
    if (s.previous_stamp_ns == stamp && index >= 0) {
      for (size_t i = 0; i < s.obstacles.size(); ++i) {
        if (s.obstacles[i].previous_cluster_index == index) {
          if (found >= 0) {valid = false;index = -1;stamp = next;return;}
          found = i;
        }
      }
    }
    stamp = next;index = found;
    if (found < 0) {valid = false;return;}
    const auto & o = s.obstacles[found];
    if (o.partial_motion_rank > 0 && sign * (-ty * o.partial_motion_vx + tx * o.partial_motion_vy) < -.10) {
      valid = false;index = -1;return;
    }
    // Once invalid, new evidence cannot revive this proposal until risk release.
    if (valid) {envelope(o);}
  }
  bool clear(const Trajectory & t, double reserve) const
  {
    if (!valid || t.poses.empty() || !std::isfinite(reserve) || reserve < 0) {return false;}
    auto previous = t.poses.front();
    for (const auto & p : t.poses) {
      const double a[2] = {along(previous), across(previous)};
      const double b[2] = {along(p), across(p)};
      const double low[2] = {lo - reserve, rear - reserve};
      const double high[2] = {hi + reserve, std::numeric_limits<double>::infinity()};
      double enter = 0, leave = 1;
      for (int k = 0; k < 2; ++k) {
        if (!std::isfinite(a[k]) || !std::isfinite(b[k])) {return false;}
        const double d = b[k] - a[k];
        if (d == 0) {
          if (a[k] < low[k] || a[k] > high[k]) {leave = -1;break;}
        } else {
          double u = (low[k] - a[k]) / d, v = (high[k] - a[k]) / d;
          if (u > v) {std::swap(u, v);}
          enter = std::max(enter, u);leave = std::min(leave, v);
        }
      }
      if (enter <= leave) {return false;}
      previous = p;
    }
    return true;
  }
};
