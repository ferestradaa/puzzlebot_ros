#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include "puzzlebot_interfaces/action/visual_servoing.hpp"

class VisualServoingActionServer : public rclcpp::Node {
public:

    using VisualServoing = puzzlebot_interfaces::action::VisualServoing;
    using GoalHandleVisualServoing = rclcpp_action::ServerGoalHandle<VisualServoing>;

    VisualServoingActionServer() : Node("visual_servoing_action_server") {
        bbox_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>( //subcribing to boundign box from cnn detection
            "pallet_detections", 10,
            std::bind(&VisualServoingActionServer::detections_callback, this, std::placeholders::_1));
        
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        action_server_ = rclcpp_action::create_server<VisualServoing>( 
            this,
            "visual_servoing",
            std::bind(&VisualServoingActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&VisualServoingActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&VisualServoingActionServer::handle_accepted, this, std::placeholders::_1));
        
        Kw_ = 0.1; //gain for angular velocity 
        Kv_ = 0.05; //gain por lineal velocity 
        image_width_ = 640;
        active_ = false;
        last_current_area_= 0.0; //state inside callback 
        
        RCLCPP_INFO(get_logger(), "Visual Servoing action server initialized");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const VisualServoing::Goal> goal) {
        
        RCLCPP_INFO(get_logger(), "Received goal request with target_area=%.0f", goal->target_area);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleVisualServoing> goal_handle) {
        
        RCLCPP_INFO(get_logger(), "Received cancel request");
        (void)goal_handle;
        active_ = false;
        geometry_msgs::msg::Twist stop;
        cmd_pub_->publish(stop);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleVisualServoing> goal_handle) {
        std::thread{std::bind(&VisualServoingActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleVisualServoing> goal_handle) {
        RCLCPP_INFO(get_logger(), "Executing visual servoing");
        
        const auto goal = goal_handle->get_goal();
        target_area_ = goal->target_area; //target area set by the client inside behavior tree
        active_ = true;
        current_goal_handle_ = goal_handle;
        
        rclcpp::Rate loop_rate(10);
        auto result = std::make_shared<VisualServoing::Result>();
        
        while (rclcpp::ok() && active_) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Goal canceled";
                goal_handle->canceled(result);
                active_ = false;
                geometry_msgs::msg::Twist stop;
                cmd_pub_->publish(stop);
                RCLCPP_INFO(get_logger(), "Goal canceled");
                return;
            }

            std::lock_guard<std::mutex> lock(error_mutex_); 
            double tolerance = target_area_* 0.05; 

            if (std::abs(last_current_area_ - target_area_) < tolerance){
                geometry_msgs::msg::Twist stop; 
                cmd_pub_ -> publish(stop); 
                result -> success = true; 
                result ->message = "Target area reached"; 
                goal_handle -> succeed(result); 
                active_ = false; 
                RCLCPP_INFO(get_logger(), "Goal succeed"); 
                return; 
            }
        }
            
            loop_rate.sleep();
        
        
        if (rclcpp::ok()) {
            result->success = true;
            result->message = "Visual servoing completed";
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal succeeded");
        }
    }

    void detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        if (!active_) {
            return;
        }
        
        if (msg->detections.empty()) {
            geometry_msgs::msg::Twist stop;
            cmd_pub_->publish(stop);
            return;
        }
        
        auto & det = msg->detections[0];
        double cx = det.bbox.center.position.x;
        double cy = det.bbox.center.position.y;
        double w = det.bbox.size_x;
        double h = det.bbox.size_y;
        
        compute_cmd(cx, cy, w, h);
    }

    void compute_cmd(double cx, double cy, double w, double h) {
        geometry_msgs::msg::Twist twist;
        double bbox_area = w * h;
        double cx_corrected = image_width_ - cx;
        double image_cx = image_width_ / 2.0;
        double ex = cx_corrected - image_cx;
        double ey = target_area_ - bbox_area;
        
        if (std::abs(ex) < 20.0) ex = 0.0;
        
        twist.angular.z = -Kw_ * ex / image_cx;
        twist.linear.x = Kv_ * ey / target_area_;
        twist.linear.x = std::clamp(twist.linear.x, -0.05, 0.15);
        twist.angular.z = std::clamp(twist.angular.z, -0.3, 0.3);
        
       // RCLCPP_INFO(get_logger(), "cx_raw=%.1f cx_corr=%.1f ex=%.1f area=%.0f",
         //           cx, cx_corrected, ex, bbox_area);
        
        cmd_pub_->publish(twist);
        
        if (current_goal_handle_) {
            auto feedback = std::make_shared<VisualServoing::Feedback>();
            feedback->current_area = bbox_area; //.action uses current area as feedback 
            current_goal_handle_->publish_feedback(feedback);

            std::lock_guard<std::mutex> lock(error_mutex_); 
            last_current_area_ = bbox_area; 
        }
    }

    rclcpp_action::Server<VisualServoing>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr bbox_sub_;
    std::shared_ptr<GoalHandleVisualServoing> current_goal_handle_;
    
    double Kw_;
    double Kv_;
    double target_area_;
    double image_width_;
    bool active_;
    double last_current_area_; 
    std::mutex error_mutex_; 
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualServoingActionServer>());
    rclcpp::shutdown();
    return 0;
}
