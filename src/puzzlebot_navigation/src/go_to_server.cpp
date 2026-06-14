#include <memory>
#include <string>
#include <map>
#include <cmath>
#include <chrono>
#include <fstream>
#include <thread>
#include <atomic>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"
#include "puzzlebot_interfaces/action/go_to.hpp"
#include "puzzlebot_interfaces/srv/plan_path.hpp"

using namespace std::chrono_literals;
using GoTo = puzzlebot_interfaces::action::GoTo;
using GoalHandle = rclcpp_action::ServerGoalHandle<GoTo>;
using PlanPath = puzzlebot_interfaces::srv::PlanPath;

struct ZonePose {
  double x, y, yaw;
};

class GoToServer : public rclcpp::Node {
public:
  GoToServer() : Node("go_to_server") {
    pos_thr_            = declare_parameter<double>("pos_threshold", 0.15);
    yaw_thr_            = declare_parameter<double>("yaw_threshold", 0.10);
    fb_rate_            = declare_parameter<double>("feedback_rate", 5.0);
    timeout_            = declare_parameter<double>("goal_timeout", 120.0);
    replan_cooldown_    = declare_parameter<double>("replan_cooldown", 1.5);
    stall_distance_thr_ = declare_parameter<double>("stall_distance_threshold", 0.15);
    stall_time_         = declare_parameter<double>("stall_time", 4.0);
    planner_retries_    = declare_parameter<int>("planner_retries", 3);
    planner_retry_wait_ = declare_parameter<double>("planner_retry_wait", 1.0);

    load_zones();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    planner_cli_ = create_client<PlanPath>("plan_path");
    path_pub_    = create_publisher<nav_msgs::msg::Path>("/path", 10);

    reached_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/pure_pursuit/goal_reached", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) goal_reached_ = true;
      });

    path_blocked_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/path_blocked", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) needs_replan_ = true;
      });

    server_ = rclcpp_action::create_server<GoTo>(
      this, "go_to",
      std::bind(&GoToServer::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GoToServer::handle_cancel,   this, std::placeholders::_1),
      std::bind(&GoToServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  void load_zones() {
    std::string pkg_path  = ament_index_cpp::get_package_share_directory("puzzlebot_navigation");
    std::string yaml_path = pkg_path + "/config/dummy_zones.yaml";
    std::ifstream f(yaml_path);
    if (!f.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open: %s", yaml_path.c_str());
      return;
    }
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (!config["zones"]) {
      RCLCPP_WARN(get_logger(), "No 'zones' key in config.yaml");
      return;
    }
    for (auto it = config["zones"].begin(); it != config["zones"].end(); ++it) {
      std::string name = it->first.as<std::string>();
      ZonePose z;
      z.x   = it->second["x"].as<double>(0.0);
      z.y   = it->second["y"].as<double>(0.0);
      z.yaw = it->second["yaw"].as<double>(0.0);
      zones_[name] = z;
    }
    if (zones_.empty()) {
      RCLCPP_WARN(get_logger(), "No valid zones loaded from dummy_zones.yaml");
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GoTo::Goal> goal)
  {
    if (goal->goal_type == GoTo::Goal::NAMED_ZONE &&
        zones_.find(goal->zone_name) == zones_.end()) {
      RCLCPP_ERROR(get_logger(), "Unknown zone: %s", goal->zone_name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> gh) {
    std::thread{std::bind(&GoToServer::execute, this, gh)}.detach();
  }

  geometry_msgs::msg::PoseStamped make_pose(double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp    = now();
    p.pose.position.x = x;
    p.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    p.pose.orientation = tf2::toMsg(q);
    return p;
  }

  geometry_msgs::msg::PoseStamped resolve_goal(std::shared_ptr<const GoTo::Goal> goal) {
    if (goal->goal_type == GoTo::Goal::NAMED_ZONE) {
      const auto & z = zones_.at(goal->zone_name);
      return make_pose(z.x, z.y, z.yaw);
    }
    auto p = goal->target_pose;
    double yaw = tf2::getYaw(p.pose.orientation);
    p.pose.position.x -= 0.55 * std::cos(yaw);
    p.pose.position.y -= 0.55 * std::sin(yaw);
    return p;
  }

  bool get_robot_pose(double & x, double & y, double & yaw) {
    try {
      auto tf = tf_buffer_->lookupTransform("map", "fork_tip_link", tf2::TimePointZero);
      x   = tf.transform.translation.x;
      y   = tf.transform.translation.y;
      yaw = tf2::getYaw(tf.transform.rotation);
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "TF lookup failed: %s", e.what());
      return false;
    }
  }

  bool call_planner(const geometry_msgs::msg::PoseStamped & target) {
    auto req  = std::make_shared<PlanPath::Request>();
    req->goal = target;

    for (int attempt = 0; attempt < planner_retries_; ++attempt) {
      auto future = planner_cli_->async_send_request(req);
      if (future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_WARN(get_logger(), "Planner timeout (attempt %d/%d)", attempt + 1, planner_retries_);
      } else {
        auto resp = future.get();
        if (resp->success) {
          path_pub_->publish(resp->path);
          return true;
        }
        RCLCPP_WARN(get_logger(), "Planner failed (attempt %d/%d): %s",
                    attempt + 1, planner_retries_, resp->message.c_str());
      }

      if (attempt + 1 < planner_retries_) {
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(planner_retry_wait_)));
      }
    }

    RCLCPP_ERROR(get_logger(), "Planner failed after %d attempts", planner_retries_);
    return false;
  }

  void execute(const std::shared_ptr<GoalHandle> gh) {
    goal_reached_     = false;
    needs_replan_     = false;
    last_replan_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    auto result = std::make_shared<GoTo::Result>();
    auto target = resolve_goal(gh->get_goal());
    double gx   = target.pose.position.x;
    double gy   = target.pose.position.y;
    double gyaw = tf2::getYaw(target.pose.orientation);

    if (!planner_cli_->wait_for_service(2s)) {
      result->success    = false;
      result->error_code = GoTo::Result::PLANNER_FAIL;
      gh->abort(result);
      return;
    }

    if (!call_planner(target)) {
      result->success    = false;
      result->error_code = GoTo::Result::PLANNER_FAIL;
      gh->abort(result);
      return;
    }

    auto fb               = std::make_shared<GoTo::Feedback>();
    rclcpp::Rate rate(fb_rate_);
    auto start            = now();
    double last_best_dist = std::numeric_limits<double>::max();
    auto last_progress_t  = now();

    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        result->success    = false;
        result->error_code = GoTo::Result::ABORTED;
        gh->canceled(result);
        return;
      }

      if ((now() - start).seconds() > timeout_) {
        result->success    = false;
        result->error_code = GoTo::Result::TIMEOUT;
        gh->abort(result);
        return;
      }

      if (needs_replan_) {
        needs_replan_ = false;
        double elapsed = (now() - last_replan_time_).seconds();
        if (elapsed >= replan_cooldown_) {
          RCLCPP_INFO(get_logger(), "Replanning...");
          last_replan_time_ = now();
          last_best_dist    = std::numeric_limits<double>::max();
          last_progress_t   = now();
          if (!call_planner(target)) {
            result->success    = false;
            result->error_code = GoTo::Result::PLANNER_FAIL;
            gh->abort(result);
            return;
          }
        } else {
          RCLCPP_DEBUG(get_logger(), "Replan cooldown active (%.1fs remaining)",
                       replan_cooldown_ - elapsed);
        }
      }

      double rx, ry, ryaw;
      if (get_robot_pose(rx, ry, ryaw)) {
        double dist = std::hypot(gx - rx, gy - ry);
        double ang  = std::abs(std::atan2(std::sin(gyaw - ryaw), std::cos(gyaw - ryaw)));

        fb->distance_remaining = static_cast<float>(dist);
        fb->angle_remaining    = static_cast<float>(ang);
        gh->publish_feedback(fb);

        if (last_best_dist - dist > stall_distance_thr_) {
          last_best_dist  = dist;
          last_progress_t = now();
        }

        double stall_elapsed = (now() - last_progress_t).seconds();
        if (stall_elapsed > stall_time_) {
          RCLCPP_WARN(get_logger(), "Stall detected (%.1fs without progress)", stall_elapsed);
          last_progress_t = now();
          last_best_dist  = dist;
          needs_replan_   = true;
        }
      }

      if (goal_reached_) break;
      rate.sleep();
    }

    result->success    = true;
    result->error_code = GoTo::Result::OK;
    gh->succeed(result);
  }

  std::map<std::string, ZonePose> zones_;
  double pos_thr_, yaw_thr_, fb_rate_, timeout_, replan_cooldown_;
  double stall_distance_thr_, stall_time_, planner_retry_wait_;
  int planner_retries_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<PlanPath>::SharedPtr         planner_cli_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_action::Server<GoTo>::SharedPtr      server_;

  std::atomic<bool> goal_reached_{false};
  std::atomic<bool> needs_replan_{false};
  rclcpp::Time last_replan_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr path_blocked_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToServer>());
  rclcpp::shutdown();
  return 0;
}