#include "puzzlebot_navigation/motion_model.hpp"

#include <algorithm>
#include <cmath>

namespace puzzlebot_navigation
{

void MotionModel::configure(
  float motion_noise_rot1,
  float motion_noise_trans,
  float alpha_rot_from_rot,
  float alpha_rot_from_trans,
  float alpha_trans_from_trans,
  float alpha_trans_from_rot,
  float odom_cov_trans_scale,
  float odom_cov_rot_scale,
  float motion_noise_min,
  float motion_noise_max)
{
  motion_noise_rot1_ = motion_noise_rot1;
  motion_noise_trans_ = motion_noise_trans;
  alpha_rot_from_rot_ = alpha_rot_from_rot;
  alpha_rot_from_trans_ = alpha_rot_from_trans;
  alpha_trans_from_trans_ = alpha_trans_from_trans;
  alpha_trans_from_rot_ = alpha_trans_from_rot;
  odom_cov_trans_scale_ = odom_cov_trans_scale;
  odom_cov_rot_scale_ = odom_cov_rot_scale;
  motion_noise_min_ = motion_noise_min;
  motion_noise_max_ = motion_noise_max;
}

Pose2D MotionModel::pose_from_odom(const nav_msgs::msg::Odometry & odom)
{
  const auto & p = odom.pose.pose.position;
  const auto & q = odom.pose.pose.orientation;
  return Pose2D{
    static_cast<float>(p.x),
    static_cast<float>(p.y),
    quaternion_to_yaw(
      static_cast<float>(q.x), static_cast<float>(q.y),
      static_cast<float>(q.z), static_cast<float>(q.w))};
}

std::optional<OdomDelta> MotionModel::compute_increment(const Pose2D & prev, const Pose2D & curr)
{
  const float dx = curr.x - prev.x;
  const float dy = curr.y - prev.y;
  const float dth = wrap_angle(curr.theta - prev.theta);

  const float delta_trans = std::hypot(dx, dy);
  const float delta_rot1 =
    (delta_trans < 1e-9f) ? 0.0f : wrap_angle(std::atan2(dy, dx) - prev.theta);
  const float delta_rot2 = wrap_angle(dth - delta_rot1);

  return OdomDelta{delta_rot1, delta_trans, delta_rot2};
}

MotionNoise MotionModel::compute_adaptive_noise(
  const OdomDelta & delta,
  const nav_msgs::msg::Odometry & odom) const
{
  const float abs_rot = std::abs(delta.rot1) + std::abs(delta.rot2);

  const auto & cov = odom.pose.covariance;
  const float cov_x = std::max(0.0f, static_cast<float>(cov[0]));
  const float cov_y = std::max(0.0f, static_cast<float>(cov[7]));
  const float cov_yaw = std::max(0.0f, static_cast<float>(cov[35]));

  const float cov_trans = std::sqrt(cov_x + cov_y);
  const float cov_rot = std::sqrt(cov_yaw);

  const float rot_noise = std::clamp(
    motion_noise_rot1_ +
    alpha_rot_from_rot_ * abs_rot +
    alpha_rot_from_trans_ * delta.trans +
    odom_cov_rot_scale_ * cov_rot,
    motion_noise_min_,
    motion_noise_max_);

  const float trans_noise = std::clamp(
    motion_noise_trans_ +
    alpha_trans_from_trans_ * delta.trans +
    alpha_trans_from_rot_ * abs_rot +
    odom_cov_trans_scale_ * cov_trans,
    motion_noise_min_,
    motion_noise_max_);

  return MotionNoise{rot_noise, trans_noise, rot_noise};
}

void MotionModel::update_dead_reckoning(const OdomDelta & delta)
{
  const float theta_mid = dead_reckoning_pose_.theta + delta.rot1;
  dead_reckoning_pose_.x += delta.trans * std::cos(theta_mid);
  dead_reckoning_pose_.y += delta.trans * std::sin(theta_mid);
  dead_reckoning_pose_.theta = wrap_angle(dead_reckoning_pose_.theta + delta.rot1 + delta.rot2);
}

}  // namespace puzzlebot_navigation
