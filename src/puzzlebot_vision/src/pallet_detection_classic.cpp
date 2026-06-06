#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

// thresholds
static constexpr int    MIN_CLUSTER_AREA   = 3000;
static constexpr int    CLUSTER_MERGE_K    = 15;
static constexpr double MAX_ANGLE_DEG      = 30.0;
static constexpr double MIN_ASPECT_RATIO   = 2.5;
static constexpr double MIN_FILL_RATIO     = 0.30;

struct Detection {
    cv::RotatedRect rect;
    int cx, cy;
    double angle;
};

class PalletLineDetector : public rclcpp::Node {
public:
    PalletLineDetector() : Node("pallet_line_detector_cv") {
        auto qos = rclcpp::QoS(1).best_effort();

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", qos,
            std::bind(&PalletLineDetector::image_callback, this, std::placeholders::_1));

        pub_det_ = create_publisher<vision_msgs::msg::Detection2DArray>("pallet_detections_", 1);
        pub_dbg_ = create_publisher<sensor_msgs::msg::Image>("pallet/debug_image", 1);

        // kernels pre-allocados
        morph_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, {5, 5});
        merge_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, {CLUSTER_MERGE_K, CLUSTER_MERGE_K});

        blue_lower_ = cv::Scalar(100, 80, 20);
        blue_upper_ = cv::Scalar(130, 255, 120);

        RCLCPP_INFO(get_logger(), "PalletLineDetector CV initilized");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
            return;
        }

        cv::Mat mask = get_blue_mask(frame);
        auto best = get_best_detection(mask);

        publish_detection(best, msg->header);

        if (pub_dbg_->get_subscription_count() > 0) {
            publish_debug(frame, best, msg->header);
        }
    }

    cv::Mat get_blue_mask(const cv::Mat & frame) {
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, blue_lower_, blue_upper_, mask);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, morph_kernel_, cv::Point(-1,-1), 3);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  morph_kernel_, cv::Point(-1,-1), 2);
        return mask;
    }

    std::optional<Detection> get_best_detection(const cv::Mat & mask) {
        cv::Mat dilated;
        cv::dilate(mask, dilated, merge_kernel_, cv::Point(-1,-1), 1);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::optional<Detection> best;
        double best_area = 0.0;

        for (const auto & cnt : contours) {
            if (cv::contourArea(cnt) < MIN_CLUSTER_AREA) continue;

            cv::Mat cluster_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
            cv::drawContours(cluster_mask, std::vector<std::vector<cv::Point>>{cnt}, -1, 255, cv::FILLED);

            cv::Mat original;
            cv::bitwise_and(mask, cluster_mask, original);
            int pixel_count = cv::countNonZero(original);

            if (pixel_count < MIN_CLUSTER_AREA) continue;

            std::vector<std::vector<cv::Point>> orig_contours;
            cv::findContours(original, orig_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            if (orig_contours.empty()) continue;

            std::vector<cv::Point> pts;
            for (const auto & c : orig_contours)
                pts.insert(pts.end(), c.begin(), c.end());

            cv::RotatedRect rect = cv::minAreaRect(pts);
            float w = rect.size.width;
            float h = rect.size.height;
            float long_side  = std::max(w, h);
            float short_side = std::min(w, h);

            if (short_side <= 0.0f) continue;
            if (long_side / short_side < MIN_ASPECT_RATIO) continue;

            double rect_area = long_side * short_side;
            if (pixel_count / rect_area < MIN_FILL_RATIO) continue;

            double angle = rect_angle(rect);
            if (std::abs(angle) > MAX_ANGLE_DEG) continue;

            if (rect_area > best_area) {
                best_area = rect_area;
                cv::Moments m = cv::moments(original, true);
                int cx = (m.m00 > 0) ? static_cast<int>(m.m10 / m.m00) : static_cast<int>(rect.center.x);
                int cy = (m.m00 > 0) ? static_cast<int>(m.m01 / m.m00) : static_cast<int>(rect.center.y);
                best = Detection{rect, cx, cy, angle};
            }
        }

        return best;
    }

    double rect_angle(const cv::RotatedRect & rect) {
        cv::Point2f box[4];
        rect.points(box);
        double max_len = -1.0;
        cv::Point2f longest_edge;
        for (int i = 0; i < 4; ++i) {
            cv::Point2f edge = box[(i + 1) % 4] - box[i];
            double len = std::hypot(edge.x, edge.y);
            if (len > max_len) { max_len = len; longest_edge = edge; }
        }
        double angle = std::atan2(longest_edge.y, longest_edge.x) * 180.0 / M_PI;
        if (angle >  90.0) angle -= 180.0;
        if (angle < -90.0) angle += 180.0;
        return angle;
    }

    void publish_detection(const std::optional<Detection> & best,
                           const std_msgs::msg::Header & header) {
        vision_msgs::msg::Detection2DArray arr;
        arr.header = header;

        if (best.has_value()) {
            vision_msgs::msg::Detection2D det;
            det.header = header;
            det.bbox.center.position.x = best->cx;
            det.bbox.center.position.y = best->cy;
            det.bbox.size_x = best->rect.size.width;
            det.bbox.size_y = best->rect.size.height;

            vision_msgs::msg::ObjectHypothesisWithPose hyp;
            hyp.hypothesis.class_id = "pallet_cv";
            hyp.hypothesis.score    = static_cast<float>(best->angle);
            det.results.push_back(hyp);
            arr.detections.push_back(det);
        }

        pub_det_->publish(arr);
    }

    void publish_debug(const cv::Mat & frame,
                       const std::optional<Detection> & best,
                       const std_msgs::msg::Header & header) {
        cv::Mat debug = frame.clone();
        if (best.has_value()) {
            cv::Point2f box[4];
            best->rect.points(box);
            for (int i = 0; i < 4; ++i)
                cv::line(debug, box[i], box[(i+1)%4], {0, 255, 0}, 2);
            cv::circle(debug, {best->cx, best->cy}, 5, {0, 0, 255}, -1);
        }
        auto dbg_msg = cv_bridge::CvImage(header, "bgr8", debug).toImageMsg();
        pub_dbg_->publish(*dbg_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_det_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_dbg_;

    cv::Mat morph_kernel_;
    cv::Mat merge_kernel_;
    cv::Scalar blue_lower_;
    cv::Scalar blue_upper_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PalletLineDetector>());
    rclcpp::shutdown();
    return 0;
}