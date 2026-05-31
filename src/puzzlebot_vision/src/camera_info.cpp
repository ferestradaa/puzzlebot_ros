// in the same node the topic from ros_deep_learning is read and stamped with camera color optical frame
// at the same time the camera info is published. This avoids creating a separate subscription to image
// for stamping and publishing camera info which contributes to light payload for the jetson
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class CameraRelay : public rclcpp::Node
{
public:
    CameraRelay() : Node("camera_relay")
    {
        this->declare_parameter<std::string>("camera_info", "camera_info_rasp.yaml");
        this->declare_parameter<std::string>("frame_id", "camera_color_optical_frame");

        frame_id_ = this->get_parameter("frame_id").as_string();

        std::string yaml_path = this->get_parameter("camera_info").as_string();
        if (yaml_path.empty() || yaml_path.front() != '/') {
            yaml_path = ament_index_cpp::get_package_share_directory("puzzlebot_vision")
                        + "/cfg/" + yaml_path;
        }

        load_camera_info(yaml_path);

        auto qos = rclcpp::SensorDataQoS().keep_last(1);

        pub_image_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", qos);
        pub_info_  = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", qos);


        
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/video_source/raw", qos,
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                auto cv_img = cv_bridge::toCvCopy(msg, msg->encoding);
                cv::flip(cv_img->image, cv_img->image, -1);
                auto flipped_msg = cv_img->toImageMsg();
                flipped_msg->header = msg->header;
                flipped_msg->header.frame_id = frame_id_;
                camera_info_.header = flipped_msg->header;
                pub_info_->publish(camera_info_);
                pub_image_->publish(std::move(*flipped_msg));
            }); 
        


        /*
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/video_source/raw", qos,
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                auto cv_img = cv_bridge::toCvCopy(msg, msg->encoding);

                msg->header.frame_id = frame_id_;
                camera_info_.header = msg->header;
                pub_info_->publish(camera_info_);
                pub_image_->publish(std::move(*msg));
            }); 
        */


        
    }

private:
    void load_camera_info(const std::string & path)
    {
        YAML::Node root = YAML::LoadFile(path);
        camera_info_.width  = root["image_width"].as<uint32_t>();
        camera_info_.height = root["image_height"].as<uint32_t>();
        camera_info_.distortion_model =
            root["distortion_model"] ?
            root["distortion_model"].as<std::string>() : "plumb_bob";

        auto load_array = [&](const char* key, auto& dest) {
            const auto& data = root[key]["data"];
            for (std::size_t i = 0; i < dest.size(); ++i)
                dest[i] = data[i].as<double>();
        };

        load_array("camera_matrix",        camera_info_.k);
        load_array("rectification_matrix", camera_info_.r);
        load_array("projection_matrix",    camera_info_.p);

        for (const auto& v : root["distortion_coefficients"]["data"])
            camera_info_.d.push_back(v.as<double>());

        camera_info_.header.frame_id = frame_id_;
    }

    std::string frame_id_;
    sensor_msgs::msg::CameraInfo camera_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_image_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraRelay>());
    rclcpp::shutdown();
    return 0;
}