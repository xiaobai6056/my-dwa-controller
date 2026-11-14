#ifndef MY_DWA_CONTROLLER__POST_LOSS_CAP_HPP_
#define MY_DWA_CONTROLLER__POST_LOSS_CAP_HPP_

#include <algorithm>
#include <cstdint>
#include <optional>

#include "my_dwa_controller/velocity_sampler.hpp"

namespace my_dwa_controller
{

class PostLossCap
{
public:
  static constexpr double close_distance = 1.30;

  bool update(int64_t now, bool has_close_prediction)
  {
    if (last_cycle_ && now < *last_cycle_) {last_close_.reset();}
    last_cycle_ = now;
    if (has_close_prediction) {
      last_close_ = now;
      return false;
    }
    return last_close_ && now - *last_close_ < 600000000;
  }

  static void limit(SamplingConfig & sampling, double measured_v)
  {
    const double reachable_min = std::max(0., measured_v - sampling.acc_v * sampling.control_dt);
    sampling.max_v = std::min(sampling.max_v, std::max(.12, reachable_min));
  }

private:
  std::optional<int64_t> last_cycle_;
  std::optional<int64_t> last_close_;
};

}  // namespace my_dwa_controller

#endif  // MY_DWA_CONTROLLER__POST_LOSS_CAP_HPP_
