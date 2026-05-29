#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unordered_map>
#include <vector>

class LandmarkPublisherNode : public rclcpp::Node
{
public:
    LandmarkPublisherNode() : Node("landmark_publisher")
    {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("puzzlebot_control");
        std::string map_path;
        this->declare_parameter("landmark_map_path", pkg_path + "/config/real_fixed.yaml");
        this->get_parameter("landmark_map_path", map_path);

        loadLandmarkMap(map_path);

        auto qos = rclcpp::QoS(1).transient_local().reliable();
        pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/landmark_poses", qos);

        publishPoses();

        RCLCPP_INFO(this->get_logger(), "Landmark poses published, %zu landmarks", landmarks_.size());
    }

private:
    void loadLandmarkMap(const std::string& path)
    {
        YAML::Node config = YAML::LoadFile(path);
        for (auto it = config["landmarks"].begin(); it != config["landmarks"].end(); ++it)
        {
            int id = it->first.as<int>();
            auto v = it->second.as<std::vector<double>>();
            landmarks_[id] = {v[0], v[1]};
        }
    }

    void publishPoses()
    {
        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        for (const auto& [id, xy] : landmarks_)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = xy[0];
            pose.position.y = xy[1];
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            msg.poses.push_back(pose);
        }

        pose_array_pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
    std::unordered_map<int, std::array<double, 2>> landmarks_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandmarkPublisherNode>());
    rclcpp::shutdown();
    return 0;
}