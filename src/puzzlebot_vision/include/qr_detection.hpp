// qr_detection.hpp
#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include "puzzlebot_control/pose_validator.hpp"

struct Qrpose {
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    std::string data;
    bool valid = false;
};

class Qr_detection {
public:
    Qr_detection(double qr_size) : qr_size_(qr_size) {}

    Qrpose estimate_qr_pose(const cv::Mat& frame, const cv::Mat& K, const cv::Mat& dist) {
        Qrpose result;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::QRCodeDetector detector;
        std::vector<cv::Point2f> corners;
        std::string decoded = detector.detectAndDecode(gray, corners);

        if (decoded.empty() || corners.size() != 4) return result;
        result.data = decoded;

        cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));

        float h = (float)(qr_size_ / 2.0);
        std::vector<cv::Point3f> obj_pts = {
            {-h,  h, 0},
            { h,  h, 0},
            { h, -h, 0},
            {-h, -h, 0}
        };

        bool ok = cv::solvePnP(obj_pts, corners, K, dist,
            result.rvec, result.tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
        result.valid = ok;
        return result;
    }

private:
    double qr_size_;
};