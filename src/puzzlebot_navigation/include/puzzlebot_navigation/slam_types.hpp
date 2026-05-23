#pragma once

#include "puzzlebot_navigation/cuda_kernels.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>

namespace puzzlebot_navigation
{

using Pose2D = slam_cuda::Pose2D;

inline constexpr float kPi = 3.14159265358979323846f;

enum class SLAMState
{
  BOOTSTRAP = 0,
  TRACKING = 1
};

struct OdomDelta
{
  float rot1{0.0f};
  float trans{0.0f};
  float rot2{0.0f};
};

struct MotionNoise
{
  float rot1{0.0f};
  float trans{0.0f};
  float rot2{0.0f};
};

inline float wrap_angle(float a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

inline float quaternion_to_yaw(float qx, float qy, float qz, float qw)
{
  const float siny_cosp = 2.0f * (qw * qz + qx * qy);
  const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

inline geometry_msgs::msg::Quaternion yaw_to_quaternion(float yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = static_cast<double>(std::sin(yaw * 0.5f));
  q.w = static_cast<double>(std::cos(yaw * 0.5f));
  return q;
}

inline const char * slam_state_name(SLAMState s)
{
  switch (s) {
    case SLAMState::BOOTSTRAP: return "BOOTSTRAP";
    case SLAMState::TRACKING: return "TRACKING";
    default: return "UNKNOWN";
  }
}

inline Pose2D relative_pose(const Pose2D & from, const Pose2D & to)
{
  const float dx = to.x - from.x;
  const float dy = to.y - from.y;
  const float c = std::cos(from.theta);
  const float s = std::sin(from.theta);

  Pose2D out{};
  out.x =  c * dx + s * dy;
  out.y = -s * dx + c * dy;
  out.theta = wrap_angle(to.theta - from.theta);
  return out;
}

inline Pose2D compose_pose(const Pose2D & base, const Pose2D & rel)
{
  const float c = std::cos(base.theta);
  const float s = std::sin(base.theta);

  Pose2D out{};
  out.x = base.x + c * rel.x - s * rel.y;
  out.y = base.y + s * rel.x + c * rel.y;
  out.theta = wrap_angle(base.theta + rel.theta);
  return out;
}

inline Pose2D transform_between_same_pose_estimates(const Pose2D & raw, const Pose2D & opt)
{
  // Returns T such that opt ~= T * raw.
  const float c = std::cos(opt.theta);
  const float s = std::sin(opt.theta);

  const float inv_raw_x = -(std::cos(raw.theta) * raw.x + std::sin(raw.theta) * raw.y);
  const float inv_raw_y =  (std::sin(raw.theta) * raw.x - std::cos(raw.theta) * raw.y);

  Pose2D t{};
  t.x = opt.x + c * inv_raw_x - s * inv_raw_y;
  t.y = opt.y + s * inv_raw_x + c * inv_raw_y;
  t.theta = wrap_angle(opt.theta - raw.theta);
  return t;
}

inline Pose2D apply_transform(const Pose2D & t, const Pose2D & p)
{
  const float c = std::cos(t.theta);
  const float s = std::sin(t.theta);

  Pose2D out{};
  out.x = t.x + c * p.x - s * p.y;
  out.y = t.y + s * p.x + c * p.y;
  out.theta = wrap_angle(t.theta + p.theta);
  return out;
}

}  // namespace puzzlebot_navigation
