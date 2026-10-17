#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <mutex>
#include <algorithm>
#include <cmath>
#include "puzzlebot_interfaces/action/visual_servoing.hpp"

class VisualServoingActionServer : public rclcpp::Node {
public:
    using VisualServoing = puzzlebot_interfaces::action::VisualServoing;
    using GoalHandleVS   = rclcpp_action::ServerGoalHandle<VisualServoing>;

    VisualServoingActionServer() : Node("visual_servoing_action_server") {
        // Angular loop works in real rad/s: Kw is rad/s per unit of normalized pixel error
        Kw_                    = declare_parameter<double>("Kw", 0.2);
        max_angular_           = declare_parameter<double>("max_angular", 0.1);
        // The robot (and the sim) turns ~5x what /cmd_vel asks for; same value as pure_pursuit_fork
        w_plant_gain_          = declare_parameter<double>("w_plant_gain", 5.0);
        Kv_                    = declare_parameter<double>("Kv", 0.06);
        // Slow approach so the detector does not lose the pallet
        max_linear_            = declare_parameter<double>("max_linear", 0.03);
        max_reverse_           = declare_parameter<double>("max_reverse", 0.03);
        min_linear_            = declare_parameter<double>("min_linear", 0.01);
        // This node is a mux bypass source (no reactive-controller ramp), so limit acceleration here
        a_linear_              = declare_parameter<double>("a_linear", 0.05);
        a_angular_             = declare_parameter<double>("a_angular", 0.1);
        // Recovery: once the pallet was tracked close enough ("armed"), a collapsed box or a lost
        // detection makes the robot back up slowly and approach again
        armed_frames_          = declare_parameter<int>("armed_frames", 20);
        armed_area_ratio_      = declare_parameter<double>("armed_area_ratio", 0.5);
        small_area_ratio_      = declare_parameter<double>("small_area_ratio", 0.25);
        small_frames_          = declare_parameter<int>("small_frames", 5);
        lost_hold_             = declare_parameter<double>("lost_hold", 0.8);
        recover_speed_         = declare_parameter<double>("recover_speed", 0.03);
        recover_time_          = declare_parameter<double>("recover_time", 3.0);
        max_recoveries_        = declare_parameter<int>("max_recoveries", 3);
        image_width_           = declare_parameter<double>("image_width", 640.0);
        max_detection_age_     = declare_parameter<double>("max_detection_age", 0.4);
        no_detection_timeout_  = declare_parameter<double>("no_detection_timeout", 5.0);

        bbox_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
            "pallet_inference_centroid", 1,
            [this](const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
                if (msg->detections.empty()) return;
                auto & d = msg->detections[0];
                std::lock_guard<std::mutex> lock(data_mutex_);
                bbox_cx_   = d.bbox.center.position.x;
                bbox_w_    = d.bbox.size_x;
                bbox_h_    = d.bbox.size_y;
                bbox_time_ = this->now();
                bbox_seq_++;
            });

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel/visual_servoing", 10);

        action_server_ = rclcpp_action::create_server<VisualServoing>(
            this, "visual_servoing",
            std::bind(&VisualServoingActionServer::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&VisualServoingActionServer::handle_cancel,   this, std::placeholders::_1),
            std::bind(&VisualServoingActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const VisualServoing::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Goal received: target_area=%.0f", goal->target_area);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleVS>) {
        publish_stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleVS> gh) {
        std::thread{[this, gh]() { execute(gh); }}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleVS> gh) {
        target_area_       = gh->get_goal()->target_area;
        frames_in_tol_     = 0;
        frames_out_of_tol_ = 0;
        prev_v_            = 0.0;
        prev_w_            = 0.0;

        int valid_frames    = 0;      // frames with the pallet seen close enough
        int small_streak    = 0;      // consecutive frames with a collapsed box
        int recoveries      = 0;
        bool armed          = false;
        rclcpp::Time recover_until(0, 0, RCL_ROS_TIME);

        auto result       = std::make_shared<VisualServoing::Result>();
        auto global_start = this->now();
        auto last_fresh   = this->now();
        const double rate_hz = 20.0;
        const double dt      = 1.0 / rate_hz;
        rclcpp::Rate rate(rate_hz);

        auto start_recovery = [&](const char * reason) {
            recoveries++;
            RCLCPP_WARN(get_logger(), "[DOCK] %s after tracking: backing up %.1fs (recovery %d/%d)",
                        reason, recover_time_, recoveries, max_recoveries_);
            recover_until = this->now() + rclcpp::Duration::from_seconds(recover_time_);
            armed = false;
            valid_frames = 0;
            small_streak = 0;
            frames_in_tol_ = 0;
            frames_out_of_tol_ = 0;
        };

        while (rclcpp::ok()) {
            if (gh->is_canceling()) {
                publish_stop();
                result->success = false;
                result->message = "Cancelled";
                gh->canceled(result);
                return;
            }

            if ((this->now() - global_start).seconds() > 60.0) {
                publish_stop();
                result->success = false;
                result->message = "Global timeout";
                gh->abort(result);
                return;
            }

            // Recovery in progress: reverse slowly, straight, then resume servoing
            if (this->now() < recover_until) {
                publish_ramped(-recover_speed_, 0.0, dt);
                rate.sleep();
                if (this->now() >= recover_until) {
                    publish_stop();
                    last_fresh = this->now();
                }
                continue;
            }

            double cx, bw, bh, age;
            uint64_t seq;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                cx  = bbox_cx_;
                bw  = bbox_w_;
                bh  = bbox_h_;
                seq = bbox_seq_;
                age = (this->now() - bbox_time_).seconds();
            }

            // only the last sample has to be recent, not a new message every tick
            bool fresh = seq > 0 && age < max_detection_age_;

            if (!fresh) {
                publish_stop();
                const double lost_for = (this->now() - last_fresh).seconds();
                if (armed && lost_for > lost_hold_) {
                    if (recoveries >= max_recoveries_) {
                        result->success = false;
                        result->message = "Recovery limit";
                        gh->abort(result);
                        return;
                    }
                    start_recovery("Detection lost");
                    continue;
                }
                if (lost_for > no_detection_timeout_) {
                    result->success = false;
                    result->message = "Lost detection";
                    gh->abort(result);
                    return;
                }
                rate.sleep();
                continue;
            }

            last_fresh = this->now();

            double bbox_area = bw * bh;
            double ex = cx - (image_width_ / 2.0);
            double ey = target_area_ - bbox_area;

            if (std::abs(ex) < 20.0) ex = 0.0;

            // Track how well the approach is going
            if (bbox_area >= armed_area_ratio_ * target_area_) {
                if (++valid_frames >= armed_frames_) armed = true;
            }
            if (armed && bbox_area < small_area_ratio_ * target_area_) {
                small_streak++;
            } else {
                small_streak = 0;
            }
            if (small_streak >= small_frames_) {
                publish_stop();
                if (recoveries >= max_recoveries_) {
                    result->success = false;
                    result->message = "Recovery limit";
                    gh->abort(result);
                    return;
                }
                start_recovery("Box collapsed");
                continue;
            }

            double tol  = target_area_ * 0.12;
            bool in_tol = bbox_area > 0.0 && std::abs(bbox_area - target_area_) < tol;

            double v = 0.0;
            double ratio = std::abs(ey) / target_area_;
            if (ratio < 0.25) {
                v = Kv_ * (ey / target_area_) * (0.3 + 0.7 * (ratio / 0.25));
            } else {
                v = Kv_ * ey / target_area_;
            }
            // Keep a minimum speed outside the tolerance band so the final approach does not stall
            if (!in_tol && std::abs(v) < min_linear_) {
                v = std::copysign(min_linear_, ey);
            }
            v = std::clamp(v, -max_reverse_, max_linear_);
            const double w = std::clamp(-Kw_ * ex / (image_width_ / 2.0), -max_angular_, max_angular_);

            if (frames_in_tol_ >= required_frames_) {
                publish_stop();
            } else {
                publish_ramped(v, w, dt);
            }

            if (in_tol) {
                frames_in_tol_++;
                frames_out_of_tol_ = 0;
            } else {
                frames_out_of_tol_++;
                if (frames_out_of_tol_ >= reset_threshold_) {
                    frames_in_tol_     = 0;
                    frames_out_of_tol_ = 0;
                }
            }

            RCLCPP_INFO(get_logger(),
                "[DOCK] area=%.0f/%.0f ex=%.0f v=%.3f w=%.3f tol_frames=%d valid=%d%s",
                bbox_area, target_area_, ex, prev_v_, prev_w_, frames_in_tol_, valid_frames,
                armed ? " armed" : "");

            auto fb          = std::make_shared<VisualServoing::Feedback>();
            fb->current_area = static_cast<float>(bbox_area);
            gh->publish_feedback(fb);

            if (frames_in_tol_ >= required_frames_) {
                publish_stop();
                result->success = true;
                result->message = "Docking completed";
                gh->succeed(result);
                return;
            }

            rate.sleep();
        }
    }

    // Move the commanded velocity toward the target with acceleration limits. Angular is handled in
    // real rad/s and divided by the plant gain on output.
    void publish_ramped(double v_target, double w_target, double dt) {
        prev_v_ += std::clamp(v_target - prev_v_, -a_linear_ * dt, a_linear_ * dt);
        prev_w_ += std::clamp(w_target - prev_w_, -a_angular_ * dt, a_angular_ * dt);
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = prev_v_;
        cmd.angular.z = prev_w_ / w_plant_gain_;
        cmd_pub_->publish(cmd);
    }

    void publish_stop() {
        prev_v_ = 0.0;
        prev_w_ = 0.0;
        cmd_pub_->publish(geometry_msgs::msg::Twist{});
    }

    rclcpp_action::Server<VisualServoing>::SharedPtr                    action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             cmd_pub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr bbox_sub_;

    double Kw_, Kv_;
    double max_angular_, w_plant_gain_;
    double max_linear_, max_reverse_, min_linear_;
    double a_linear_, a_angular_;
    int    armed_frames_, small_frames_, max_recoveries_;
    double armed_area_ratio_, small_area_ratio_, lost_hold_, recover_speed_, recover_time_;
    double prev_v_ = 0.0, prev_w_ = 0.0;
    double image_width_;
    double target_area_;
    double max_detection_age_;
    double no_detection_timeout_;

    double       bbox_cx_   = 0.0;
    double       bbox_w_    = 0.0;
    double       bbox_h_    = 0.0;
    rclcpp::Time bbox_time_ {0, 0, RCL_ROS_TIME};
    std::mutex   data_mutex_;

    uint64_t bbox_seq_ = 0;

    int frames_in_tol_     = 0;
    int frames_out_of_tol_ = 0;

    const int required_frames_ = 20;
    const int reset_threshold_ = 5;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualServoingActionServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}