// pure_pursuit_node.cpp
#include "puzzlebot_control/pure_pursuit_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

static double quaternionToYaw(const geometry_msgs::msg::Quaternion& q)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit_node"), path_received_(false)
    {
        Params p;
        p.ld_min       = 0.3;
        p.ld_k         = 0.5;
        p.v_max        = 0.2;
        p.k_curvature  = 2.0;
        p.k_crosstrack = 0.4;
        p.wheelbase    = 0.18;
        p.goal_tol     = 0.12;
        p.stop_dist    = 0.5;
        p.a_max        = 0.3;

        controller_ = std::make_unique<PurePursuitController>(p);

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // ── Subscriber del path ───────────────────────────────────────────────
        // nav_msgs/Path contiene un vector de PoseStamped.
        // El controlador se activa solo cuando haya path cargado.
        // QoS transient_local: si el publisher ya mandó el path antes de que
        // este nodo arranque, lo recibimos igual (como un "latched topic" en ROS1).
        auto qos_path = rclcpp::QoS(1).transient_local();
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "path", qos_path,
            std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Pure Pursuit Node iniciado. Esperando path...");
    }

private:
    // ── Callback del path ─────────────────────────────────────────────────────
    // Se llama una vez (o cada vez que el planner publique un path nuevo).
    // Convierte los PoseStamped a Point2D y los carga en el controlador.
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            RCLCPP_WARN(get_logger(), "Path recibido está vacío, ignorando.");
            return;
        }

        std::vector<Point2D> path;
        path.reserve(msg->poses.size());

        for (const auto& pose_stamped : msg->poses) {
            Point2D pt;
            pt.x = pose_stamped.pose.position.x;
            pt.y = pose_stamped.pose.position.y;
            path.push_back(pt);
        }

        controller_->setPath(path);
        path_received_ = true;

        RCLCPP_INFO(get_logger(), "Path cargado: %zu waypoints.", path.size());
    }

    // ── Callback de odometría ─────────────────────────────────────────────────
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Guardamos silencio hasta tener un path
        if (!path_received_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "Odometría recibida pero aún no hay path.");
            return;
        }

        if (controller_->goalReached()) {
            cmd_pub_->publish(geometry_msgs::msg::Twist{});
            RCLCPP_INFO_ONCE(get_logger(), "Meta alcanzada.");
            return;
        }

        RobotState state;
        state.x     = msg->pose.pose.position.x;
        state.y     = msg->pose.pose.position.y;
        state.theta = quaternionToYaw(msg->pose.pose.orientation);
        state.v     = msg->twist.twist.linear.x;

        const ControlOutput cmd = controller_->compute(state);

        geometry_msgs::msg::Twist twist;
        twist.linear.x  = cmd.v;
        twist.angular.z = cmd.omega;
        cmd_pub_->publish(twist);

        RCLCPP_DEBUG(get_logger(),
            "v=%.3f  ω=%.3f  cte=%.3f  ld=%.3f",
            cmd.v, cmd.omega,
            controller_->getCrossTrackError(),
            controller_->getCurrentLd());
    }

    bool path_received_;

    std::unique_ptr<PurePursuitController> controller_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr   odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr       path_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}