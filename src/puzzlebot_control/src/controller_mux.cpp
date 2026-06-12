#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <unordered_set>
#include <string>
#include <unordered_map>
#include <chrono>

static const std::unordered_set<std::string> VALID_SOURCES = {
    "pure_pursuit",
    "visual_servoing",
    "drive_raw",
};

class ControllerMux : public rclcpp::Node
{
public:
    ControllerMux() : Node("controller_mux")
    {
        declare_parameter("default_source",  "pure_pursuit");
        declare_parameter("output_topic",    "/cmd_vel_desired");
        declare_parameter("bypass_topic",    "/cmd_vel");
        declare_parameter("source_topic",    "/cmd_vel/active_source");
        declare_parameter("timeout_sec",     0.5);
        declare_parameter("bypass_sources",  std::vector<std::string>{"visual_servoing"});

        active_source_ = get_parameter("default_source").as_string();
        double timeout = get_parameter("timeout_sec").as_double();
        timeout_ = rclcpp::Duration::from_seconds(timeout);

        auto bypass_list = get_parameter("bypass_sources").as_string_array();
        bypass_sources_ = std::unordered_set<std::string>(bypass_list.begin(), bypass_list.end());

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            get_parameter("output_topic").as_string(), 10);

        bypass_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            get_parameter("bypass_topic").as_string(), 10);

        source_sub_ = create_subscription<std_msgs::msg::String>(
            get_parameter("source_topic").as_string(), 10,
            std::bind(&ControllerMux::source_cb, this, std::placeholders::_1));

        for (auto & name : VALID_SOURCES) {
            latest_cmds_[name] = geometry_msgs::msg::Twist{};
            last_recv_[name]   = rclcpp::Time(0, 0, RCL_ROS_TIME);
            auto sub = create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel/" + name, 10,
                [this, name](const geometry_msgs::msg::Twist::SharedPtr msg) {
                    cmd_cb(name, msg);
                });
            subs_.push_back(sub);
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ControllerMux::loop, this));

        RCLCPP_INFO(get_logger(), "controller_mux ready, default source: %s", active_source_.c_str());
    }

private:
    void cmd_cb(const std::string & name, const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        latest_cmds_[name] = *msg;
        last_recv_[name]   = now();
    }

    void source_cb(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string & requested = msg->data;
        if (VALID_SOURCES.find(requested) == VALID_SOURCES.end()) {
            RCLCPP_WARN(get_logger(), "unknown source '%s', ignoring", requested.c_str());
            return;
        }
        if (requested == active_source_) return;

        if (bypass_sources_.count(active_source_)) {
            bypass_pub_->publish(geometry_msgs::msg::Twist{});
        }

        RCLCPP_INFO(get_logger(), "source: %s -> %s", active_source_.c_str(), requested.c_str());
        active_source_ = requested;
    }

    void loop()
    {
        auto t = now();
        auto it_recv = last_recv_.find(active_source_);
        bool timed_out = false;
        if (it_recv != last_recv_.end()) {
            double elapsed = (t - it_recv->second).seconds();
            timed_out = (it_recv->second.nanoseconds() == 0) || (elapsed > timeout_.seconds());
        }

        bool is_bypass = bypass_sources_.count(active_source_) > 0;
        auto & pub = is_bypass ? bypass_pub_ : cmd_pub_;

        if (timed_out) {
            pub->publish(geometry_msgs::msg::Twist{});
            return;
        }

        pub->publish(latest_cmds_[active_source_]);
    }

    std::string active_source_;
    rclcpp::Duration timeout_{0, 0};
    std::unordered_set<std::string> bypass_sources_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr bypass_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  source_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> subs_;
    std::unordered_map<std::string, geometry_msgs::msg::Twist> latest_cmds_;
    std::unordered_map<std::string, rclcpp::Time>              last_recv_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerMux>());
    rclcpp::shutdown();
    return 0;
}