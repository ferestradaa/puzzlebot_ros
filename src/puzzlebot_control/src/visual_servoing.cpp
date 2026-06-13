#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include "puzzlebot_interfaces/action/visual_servoing.hpp"

class VisualServoingActionServer : public rclcpp::Node {
public:
    using VisualServoing     = puzzlebot_interfaces::action::VisualServoing;
    using GoalHandleVS       = rclcpp_action::ServerGoalHandle<VisualServoing>;
    enum class DockPhase     { ALIGN_YAW, DOCK };

    VisualServoingActionServer() : Node("visual_servoing_action_server") {
        Kw_                   = declare_parameter<double>("Kw", 0.1);
        Kv_                   = declare_parameter<double>("Kv", 0.09);
        Kw_yaw_               = declare_parameter<double>("Kw_yaw", 0.6);
        image_width_          = declare_parameter<double>("image_width", 640.0);
        yaw_align_threshold_  = declare_parameter<double>("yaw_align_threshold", 0.15);
        yaw_converge_thr_     = declare_parameter<double>("yaw_converge_threshold", 0.05);
        phase_timeout_        = declare_parameter<double>("phase_timeout", 10.0);

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
            });

        qr_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/qr_pose_camera", 1,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                double tx = msg->pose.position.x;
                double tz = msg->pose.position.z;
                if (tz < 0.01) return;
                std::lock_guard<std::mutex> lock(data_mutex_);
                yaw_error_ = std::atan2(tx, tz);
                qr_time_   = this->now();
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
        RCLCPP_INFO(get_logger(), "Goal recibido: target_area=%.0f", goal->target_area);
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
        target_area_          = gh->get_goal()->target_area;
        frames_yaw_aligned_   = 0;
        frames_in_tol_        = 0;
        frames_out_of_tol_    = 0;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            double age = (this->now() - qr_time_).seconds();
            bool qr_fresh = age < 1.0;
            phase_ = (qr_fresh && std::abs(yaw_error_) > yaw_align_threshold_)
                     ? DockPhase::ALIGN_YAW
                     : DockPhase::DOCK;
        }

        RCLCPP_INFO(get_logger(), "Iniciando fase: %s",
            phase_ == DockPhase::ALIGN_YAW ? "ALIGN_YAW" : "DOCK");

        auto result       = std::make_shared<VisualServoing::Result>();
        auto global_start = this->now();
        auto phase_start  = this->now();
        rclcpp::Rate rate(20);

        while (rclcpp::ok()) {
            if (gh->is_canceling()) {
                publish_stop();
                result->success = false;
                result->message = "Cancelado";
                gh->canceled(result);
                return;
            }

            if ((this->now() - global_start).seconds() > 60.0) {
                publish_stop();
                result->success = false;
                result->message = "Timeout global";
                gh->abort(result);
                return;
            }

            if ((this->now() - phase_start).seconds() > phase_timeout_) {
                publish_stop();
                result->success = false;
                result->message = "Timeout de fase: " +
                    std::string(phase_ == DockPhase::ALIGN_YAW ? "ALIGN_YAW" : "DOCK");
                gh->abort(result);
                return;
            }

            double yaw_err, cx, bw, bh;
            rclcpp::Time qr_t, bbox_t;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                yaw_err = yaw_error_;
                cx      = bbox_cx_;
                bw      = bbox_w_;
                bh      = bbox_h_;
                qr_t    = qr_time_;
                bbox_t  = bbox_time_;
            }

            bool qr_fresh   = (this->now() - qr_t).seconds()   < 0.5;
            bool bbox_fresh = (this->now() - bbox_t).seconds()  < 0.5;

            if (phase_ == DockPhase::ALIGN_YAW) {
                if (!qr_fresh) {
                    publish_stop();
                    rate.sleep();
                    continue;
                }

                phase_start = this->now();

                geometry_msgs::msg::Twist cmd;
                if (std::abs(yaw_err) > yaw_converge_thr_) {
                    cmd.angular.z = std::clamp(-Kw_yaw_ * yaw_err, -0.3, 0.3);
                    cmd.linear.x  = 0.0;
                    frames_yaw_aligned_ = 0;

                } else {
                    publish_stop();
                    frames_yaw_aligned_++;
                    RCLCPP_INFO(get_logger(), "ALIGN_YAW convergiendo, frames=%d", frames_yaw_aligned_);
                }
                cmd_pub_->publish(cmd);

                if (frames_yaw_aligned_ >= required_yaw_frames_) {
                    phase_      = DockPhase::DOCK;
                    phase_start = this->now();
                    frames_in_tol_     = 0;
                    frames_out_of_tol_ = 0;
                    RCLCPP_INFO(get_logger(), "Transicion a DOCK");
                }

            } else {
                if (!bbox_fresh) {
                    publish_stop();
                    rate.sleep();
                    continue;
                }

                phase_start = this->now();

                double bbox_area = bw * bh;
                double ex        = cx - (image_width_ / 2.0);
                double ey        = target_area_ - bbox_area;

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

                double tol       = target_area_ * 0.12;
                bool in_tol      = bbox_area > 0.0 && std::abs(bbox_area - target_area_) < tol;

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

                RCLCPP_INFO(get_logger(),
                    "[DOCK] ex=%.1f ey=%.1f area=%.0f lin=%.3f ang=%.3f tol_frames=%d",
                    ex, ey, bbox_area, cmd.linear.x, cmd.angular.z, frames_in_tol_);

                auto fb = std::make_shared<VisualServoing::Feedback>();
                fb->current_area = static_cast<float>(bbox_area);
                gh->publish_feedback(fb);

                if (frames_in_tol_ >= required_frames_) {
                    publish_stop();
                    result->success = true;
                    result->message = "Docking completado";
                    gh->succeed(result);
                    return;
                }
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
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    qr_sub_;

    double Kw_, Kv_, Kw_yaw_;
    double image_width_;
    double target_area_;
    double yaw_align_threshold_;
    double yaw_converge_thr_;
    double phase_timeout_;

    DockPhase phase_ = DockPhase::DOCK;

    double         yaw_error_ = 0.0;
    double         bbox_cx_   = 0.0;
    double         bbox_w_    = 0.0;
    double         bbox_h_    = 0.0;
    rclcpp::Time   qr_time_   {0, 0, RCL_ROS_TIME};
    rclcpp::Time   bbox_time_ {0, 0, RCL_ROS_TIME};
    std::mutex     data_mutex_;

    int frames_yaw_aligned_ = 0;
    int frames_in_tol_      = 0;
    int frames_out_of_tol_  = 0;

    const int required_yaw_frames_ = 10;
    const int required_frames_     = 20;
    const int reset_threshold_     = 5;
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