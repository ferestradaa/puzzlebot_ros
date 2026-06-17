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
        Kw_                    = declare_parameter<double>("Kw", 0.1);
        Kv_                    = declare_parameter<double>("Kv", 0.15);
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

        auto result       = std::make_shared<VisualServoing::Result>();
        auto global_start = this->now();
        auto last_fresh    = this->now();
        rclcpp::Rate rate(20);

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

            // ya no exigimos un mensaje nuevo cada tick, solo que el ultimo dato no este viejo
            bool fresh = seq > 0 && age < max_detection_age_;

            if (!fresh) {
                publish_stop();
                if ((this->now() - last_fresh).seconds() > no_detection_timeout_) {
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

            geometry_msgs::msg::Twist cmd;
            double ratio = std::abs(ey) / target_area_;
            if (ratio < 0.25) {
                cmd.linear.x = Kv_ * (ey / target_area_) * (0.3 + 0.7 * (ratio / 0.25));
            } else {
                cmd.linear.x = Kv_ * ey / target_area_;
            }
            cmd.linear.x  = std::clamp(cmd.linear.x, -0.15, 0.3);
            cmd.angular.z = std::clamp(-Kw_ * ex / (image_width_ / 2.0), -0.5, 0.5);

            double tol  = target_area_ * 0.12;
            bool in_tol = bbox_area > 0.0 && std::abs(bbox_area - target_area_) < tol;

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

            if (frames_in_tol_ < required_frames_) {
                cmd_pub_->publish(cmd);
            }

            RCLCPP_INFO(get_logger(), "[DOCK] area=%.0f ex=%.1f tol_frames=%d",
                bbox_area, ex, frames_in_tol_);

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

    void publish_stop() {
        cmd_pub_->publish(geometry_msgs::msg::Twist{});
    }

    rclcpp_action::Server<VisualServoing>::SharedPtr                    action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             cmd_pub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr bbox_sub_;

    double Kw_, Kv_;
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