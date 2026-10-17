#include <memory>
#include <string>
#include <map>
#include <cmath>
#include <chrono>
#include <fstream>
#include <thread>
#include <atomic>
#include <limits>
#include <vector>
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
    // limites de la busqueda del standoff frente al pallet (no es un valor fijo)
    standoff_max_       = declare_parameter<double>("standoff_max", 0.8);
    standoff_min_       = declare_parameter<double>("standoff_min", 0.3);
    standoff_step_      = declare_parameter<double>("standoff_step", 0.1);

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

  // Para un named zone hay un solo candidato. Para una pose (QR) se generan candidatos
  // sobre la normal de la superficie, del standoff mas lejano al mas cercano.
  std::vector<geometry_msgs::msg::PoseStamped> resolve_goal(
    std::shared_ptr<const GoTo::Goal> goal)
  {
    if (goal->goal_type == GoTo::Goal::NAMED_ZONE) {
      const auto & z = zones_.at(goal->zone_name);
      return {make_pose(z.x, z.y, z.yaw)};
    }

    const auto & qr = goal->target_pose;
    const double qx = qr.pose.position.x;
    const double qy = qr.pose.position.y;

    // Normal de la superficie: eje z del marco del QR (solvePnP) rotado a map, proyectado en xy
    tf2::Quaternion q;
    tf2::fromMsg(qr.pose.orientation, q);
    tf2::Vector3 n = tf2::quatRotate(q, tf2::Vector3(0, 0, 1));
    double nx = n.x(), ny = n.y();

    double rx, ry, ryaw;
    const bool have_robot = get_robot_pose(rx, ry, ryaw);

    double norm = std::hypot(nx, ny);
    if (norm < 0.2 && have_robot) {
      // QR visto casi de frente a la camara/vertical: la normal horizontal no es confiable,
      // usar la direccion robot -> QR
      nx = qx - rx;
      ny = qy - ry;
      norm = std::hypot(nx, ny);
    }
    if (norm < 1e-6) {
      return {qr};
    }
    nx /= norm;
    ny /= norm;

    // El signo de la normal depende de la convencion del QR: forzarlo a apuntar del robot al QR
    if (have_robot && (nx * (qx - rx) + ny * (qy - ry)) < 0.0) {
      nx = -nx;
      ny = -ny;
    }

    const double yaw = std::atan2(ny, nx);
    std::vector<geometry_msgs::msg::PoseStamped> out;
    for (double d = standoff_max_; d >= standoff_min_ - 1e-9; d -= standoff_step_) {
      out.push_back(make_pose(qx - d * nx, qy - d * ny, yaw));
    }
    return out;
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

  bool call_planner(const geometry_msgs::msg::PoseStamped & target, int attempts = -1) {
    if (attempts < 0) attempts = planner_retries_;
    auto req  = std::make_shared<PlanPath::Request>();
    req->goal = target;

    for (int attempt = 0; attempt < attempts; ++attempt) {
      auto future = planner_cli_->async_send_request(req);
      if (future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_WARN(get_logger(), "Planner timeout (attempt %d/%d)", attempt + 1, attempts);
      } else {
        auto resp = future.get();
        if (resp->success) {
          path_pub_->publish(resp->path);
          return true;
        }
        RCLCPP_WARN(get_logger(), "Planner failed (attempt %d/%d): %s",
                    attempt + 1, attempts, resp->message.c_str());
      }

      if (attempt + 1 < attempts) {
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(planner_retry_wait_)));
      }
    }

    RCLCPP_ERROR(get_logger(), "Planner failed after %d attempts", attempts);
    return false;
  }

  void execute(const std::shared_ptr<GoalHandle> gh) {
    goal_reached_     = false;
    needs_replan_     = false;
    last_replan_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    auto result = std::make_shared<GoTo::Result>();
    auto candidates = resolve_goal(gh->get_goal());

    if (!planner_cli_->wait_for_service(2s)) {
      result->success    = false;
      result->error_code = GoTo::Result::PLANNER_FAIL;
      gh->abort(result);
      return;
    }

    // Primer candidato con camino; el ultimo usa todos los reintentos
    geometry_msgs::msg::PoseStamped target;
    bool planned = false;
    for (size_t i = 0; i < candidates.size() && !planned; ++i) {
      const bool last = (i + 1 == candidates.size());
      if (call_planner(candidates[i], last ? -1 : 1)) {
        target  = candidates[i];
        planned = true;
        if (candidates.size() > 1) {
          RCLCPP_INFO(get_logger(), "Standoff candidate %zu/%zu selected (%.2f, %.2f)",
                      i + 1, candidates.size(),
                      target.pose.position.x, target.pose.position.y);
        }
      }
    }
    if (!planned) {
      result->success    = false;
      result->error_code = GoTo::Result::PLANNER_FAIL;
      gh->abort(result);
      return;
    }
    double gx   = target.pose.position.x;
    double gy   = target.pose.position.y;
    double gyaw = tf2::getYaw(target.pose.orientation);

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
  double standoff_max_, standoff_min_, standoff_step_;
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