// qr_detection.hpp
#pragma once
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <optional>

struct Qrpose {
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    std::string data;
    bool valid = false;
};

class Qr_detection {
public:
    Qr_detection(double qr_size, int confirmation_threshold = 3) 
        : qr_size_(qr_size), 
          confirmation_threshold_(confirmation_threshold),
          detection_count_(0) {
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    }

    Qrpose estimate_qr_pose(const cv::Mat& frame, const cv::Mat& K, const cv::Mat& dist) {
        Qrpose result;
        cv::Mat gray;
        if (frame.channels() == 3) {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = frame.clone();
        }

        cv::Mat resized = gray;
        if (gray.cols > 1280) {
            double scale = 1280.0 / gray.cols;
            cv::resize(gray, resized, cv::Size(), scale, scale);
        }

        cv::equalizeHist(resized, resized);

        auto variants = preprocess_variants(resized);
        
        std::unordered_map<std::string, std::vector<cv::Point2f>> seen;
        
        for (const auto& variant : variants) {
            zbar::Image zimg(variant.cols, variant.rows, "Y800", variant.data, variant.cols * variant.rows);
            int n = scanner_.scan(zimg);
            
            if (n > 0) {
                for (auto sym = zimg.symbol_begin(); sym != zimg.symbol_end(); ++sym) {
                    std::string data = sym->get_data();
                    
                    if (seen.find(data) == seen.end() && sym->get_location_size() == 4) {
                        std::vector<cv::Point2f> raw_corners;
                        for (int i = 0; i < 4; i++) {
                            raw_corners.emplace_back((float)sym->get_location_x(i), (float)sym->get_location_y(i));
                        }
                        seen[data] = sort_corners(raw_corners);
                    }
                }
            }
        }

        if (seen.empty()) {
            detection_count_ = 0;
            last_detection_.clear();
            return result;
        }

        std::string current_data = seen.begin()->first;
        
        if (last_detection_ == current_data) {
            detection_count_++;
        } else {
            last_detection_ = current_data;
            detection_count_ = 1;
        }

        if (detection_count_ < confirmation_threshold_) {
            return result;
        }

        auto sorted_corners = seen.begin()->second;
        
        float h = (float)(qr_size_ / 2.0);
        std::vector<cv::Point3f> obj_pts = {
            {-h,  h, 0},
            { h,  h, 0},
            { h, -h, 0},
            {-h, -h, 0}
        };

        bool ok = cv::solvePnP(obj_pts, sorted_corners, K, dist,
                               result.rvec, result.tvec, false, cv::SOLVEPNP_ITERATIVE);
        
        result.valid = ok;
        result.data = current_data;
        return result;
    }

    int get_detection_count() const { return detection_count_; }
    std::string get_last_detection() const { return last_detection_; }

private:
    std::vector<cv::Mat> preprocess_variants(const cv::Mat& gray) {
        std::vector<cv::Mat> variants;
        
        variants.push_back(gray.clone());
        
        cv::Mat clahe_img;
        auto clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(gray, clahe_img);
        variants.push_back(clahe_img);
        
        cv::Mat sharpened;
        cv::Mat kernel = (cv::Mat_<float>(3, 3) << -1, -1, -1, -1, 9, -1, -1, -1, -1);
        cv::filter2D(gray, sharpened, -1, kernel);
        variants.push_back(sharpened);
        
        cv::Mat bilateral;
        cv::bilateralFilter(gray, bilateral, 9, 75, 75);
        variants.push_back(bilateral);
        
        cv::Mat otsu;
        cv::threshold(gray, otsu, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        variants.push_back(otsu);
        
        cv::Mat adaptive1;
        cv::adaptiveThreshold(gray, adaptive1, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, 
                             cv::THRESH_BINARY, 11, 2);
        variants.push_back(adaptive1);
        
        cv::Mat adaptive2;
        cv::adaptiveThreshold(gray, adaptive2, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                             cv::THRESH_BINARY, 15, 3);
        variants.push_back(adaptive2);
        
        cv::Mat closed;
        cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(gray, closed, cv::MORPH_CLOSE, morph_kernel);
        variants.push_back(closed);
        
        return variants;
    }

    std::vector<cv::Point2f> sort_corners(const std::vector<cv::Point2f>& raw_corners) {
        std::vector<cv::Point2f> sorted_corners(4);
        
        auto sort_by_sum = [](const cv::Point2f& a, const cv::Point2f& b) { 
            return (a.x + a.y) < (b.x + b.y); 
        };
        auto sort_by_diff = [](const cv::Point2f& a, const cv::Point2f& b) { 
            return (a.x - a.y) > (b.x - b.y); 
        };
        
        std::vector<cv::Point2f> pts = raw_corners;
        
        sorted_corners[0] = *std::min_element(pts.begin(), pts.end(), sort_by_sum);
        sorted_corners[2] = *std::max_element(pts.begin(), pts.end(), sort_by_sum);
        sorted_corners[1] = *std::max_element(pts.begin(), pts.end(), sort_by_diff);
        sorted_corners[3] = *std::min_element(pts.begin(), pts.end(), sort_by_diff);
        
        return sorted_corners;
    }

    double qr_size_;
    int confirmation_threshold_;
    int detection_count_;
    std::string last_detection_;
    zbar::ImageScanner scanner_;
};