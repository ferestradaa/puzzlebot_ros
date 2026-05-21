// qr_detection.hpp
#pragma once
#include <opencv2/opencv.hpp>
#include <zbar.h>

struct Qrpose {
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    std::string data;
    bool valid = false;
};

class Qr_detection {
public:
    Qr_detection(double qr_size) : qr_size_(qr_size) {
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    }

    Qrpose estimate_qr_pose(const cv::Mat& frame, const cv::Mat& K, const cv::Mat& dist) {
        Qrpose result;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        zbar::Image zimg(gray.cols, gray.rows, "Y800",
                         gray.data, gray.cols * gray.rows);
        int n = scanner_.scan(zimg);
        if (n <= 0) return result;

        auto sym = zimg.symbol_begin();
        if (sym == zimg.symbol_end()) return result;

        result.data = sym->get_data();

        int loc_size = sym->get_location_size();
        if (loc_size < 4) return result;

        std::vector<cv::Point2f> corners;
        for (int i = 0; i < 4; i++) {
            corners.emplace_back(
                (float)sym->get_location_x(i),
                (float)sym->get_location_y(i)
            );
        }

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
    zbar::ImageScanner scanner_;
};