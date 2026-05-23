#pragma once

#include "puzzlebot_navigation/slam_types.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <optional>

namespace puzzlebot_navigation
{

class MotionModel
{
public:
  void configure(
    float motion_noise_rot1,
    float motion_noise_trans,
    float alpha_rot_from_rot,
    float alpha_rot_from_trans,
    float alpha_trans_from_trans,
    float alpha_trans_from_rot,
    float odom_cov_trans_scale,
    float odom_cov_rot_scale,
    float motion_noise_min,
    float motion_noise_max);

  static Pose2D pose_from_odom(const nav_msgs::msg::Odometry & odom);
  static std::optional<OdomDelta> compute_increment(const Pose2D & prev, const Pose2D & curr);

  MotionNoise compute_adaptive_noise(
    const OdomDelta & delta,
    const nav_msgs::msg::Odometry & odom) const;

  void reset_dead_reckoning(const Pose2D & pose) { dead_reckoning_pose_ = pose; }
  void update_dead_reckoning(const OdomDelta & delta);
  const Pose2D & dead_reckoning_pose() const { return dead_reckoning_pose_; }

private:
  Pose2D dead_reckoning_pose_{};

  float motion_noise_rot1_{0.03f};
  float motion_noise_trans_{0.03f};
  float alpha_rot_from_rot_{0.08f};
  float alpha_rot_from_trans_{0.03f};
  float alpha_trans_from_trans_{0.08f};
  float alpha_trans_from_rot_{0.03f};
  float odom_cov_trans_scale_{1.0f};
  float odom_cov_rot_scale_{1.0f};
  float motion_noise_min_{0.005f};
  float motion_noise_max_{0.35f};
};

}  // namespace puzzlebot_navigation
