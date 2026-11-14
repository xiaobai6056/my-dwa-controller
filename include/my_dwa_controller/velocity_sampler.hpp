#ifndef MY_DWA_CONTROLLER__VELOCITY_SAMPLER_HPP_
#define MY_DWA_CONTROLLER__VELOCITY_SAMPLER_HPP_

#include <cstddef>
#include <vector>

#include "my_dwa_controller/types.hpp"

namespace my_dwa_controller
{

struct SamplingConfig
{
  double max_v = 0.22;  // 非负前进速度上限，单位 m/s；不采样倒车速度。
  double max_w = 1.0;  // 非负角速度幅值上限，单位 rad/s。
  double acc_v = 0.5;  // 非负线加速度幅值上限，单位 m/s^2。
  double acc_w = 2.0;  // 非负角加速度幅值上限，单位 rad/s^2。
  double control_dt = 0.05;  // 正数控制周期，单位 s；与预测积分步长不同。
  std::size_t v_samples = 5;  // 基础采样数，至少为 2，并包含两个端点。
  std::size_t w_samples = 11;
};

/// 在加速度约束与静态速度限制的交集中均匀采样。
/// 保留两个端点和可达的零值，各维度分别去除相等的采样值。
/// 退化区间只生成一个值；补零可能增加基础采样数。
/// 结果按 v 升序、同一 v 下按 omega 升序排列；空交集返回空集合。
/// 当前速度允许超出静态限制，但必须有限；本函数不截断当前速度。
/// 当前速度或配置含非有限值、限制为负、control_dt 非正，或任一维度
/// 的采样数小于 2（包括退化区间）时，抛出 std::invalid_argument。
std::vector<Velocity2D> sampleVelocities(
  const Velocity2D & current, const SamplingConfig & config);

}  // 命名空间 my_dwa_controller

#endif  // 头文件保护 MY_DWA_CONTROLLER__VELOCITY_SAMPLER_HPP_
