// qr_detection.hpp
#pragma once
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <vector>
#include <string>
#include <numeric>

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
        if (frame.channels() == 3) {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = frame.clone();
        }

        zbar::Image zimg(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);
        int n = scanner_.scan(zimg);
        if (n <= 0) return result;

        auto sym = zimg.symbol_begin();
        if (sym == zimg.symbol_end()) return result;

        result.data = sym->get_data();
        if (sym->get_location_size() != 4) return result;

        std::vector<cv::Point2f> raw_corners;
        for (int i = 0; i < 4; i++) {
            raw_corners.emplace_back((float)sym->get_location_x(i), (float)sym->get_location_y(i));
        }

        // --- SOLUCIÓN: Ordenar las esquinas consistentemente ---
        // Buscamos un orden estricto: [Arriba-Izquierda, Arriba-Derecha, Abajo-Derecha, Abajo-Izquierda]
        std::vector<cv::Point2f> sorted_corners(4);
        
        // Calcular centroide para ayudar a definir cuadrantes
        cv::Point2f center(0, 0);
        for (const auto& pt : raw_corners) center += pt;
        center *= 0.25f;

        // Clasificar según su posición respecto al centroide y sus sumas/restas
        std::vector<cv::Point2f> top, bottom;
        for (const auto& pt : raw_corners) {
            if (pt.y < center.y) top.push_back(pt);
            else bottom.push_back(pt);
        }

        // Si la rotación es muy extrema (ej. 45 grados), la separación simple por Y puede fallar.
        // Un método universal más robusto es usar la suma y resta de coordenadas (Métrica de bounding box):
        // Arriba-Izquierda tiene la menor suma (x+y). Abajo-Derecha tiene la mayor suma (x+y).
        // Arriba-Derecha tiene la menor resta (y-x). Abajo-Izquierda tiene la mayor resta (y-x).
        
        auto sort_by_sum = [](const cv::Point2f& a, const cv::Point2f& b) { return (a.x + a.y) < (b.x + b.y); };
        auto sort_by_diff = [](const cv::Point2f& a, const cv::Point2f& b) { return (a.x - a.y) > (b.x - b.y); };

        // Copia local para ordenar libremente
        std::vector<cv::Point2f> pts = raw_corners;
        
        // 1. Arriba-Izquierda: Mínimo (x + y)
        sorted_corners[0] = *std::min_element(pts.begin(), pts.end(), sort_by_sum);
        // 2. Abajo-Derecha: Máximo (x + y)
        sorted_corners[2] = *std::max_element(pts.begin(), pts.end(), sort_by_sum);
        // 3. Arriba-Derecha: Máximo (x - y)
        sorted_corners[1] = *std::max_element(pts.begin(), pts.end(), sort_by_diff);
        // 4. Abajo-Izquierda: Mínimo (x - y)
        sorted_corners[3] = *std::min_element(pts.begin(), pts.end(), sort_by_diff);

        // Omitimos cornerSubPix temporalmente para evaluar estabilidad pura de ZBar.
        // Si la necesitas, actívala abajo, pero a veces empeora la distorsión en ángulos agudos.
        // cv::cornerSubPix(gray, sorted_corners, cv::Size(3,3), cv::Size(-1,-1), ...);

        // Puntos del objeto mapeados 1:1 con nuestro sorted_corners
        float h = (float)(qr_size_ / 2.0);
        std::vector<cv::Point3f> obj_pts = {
            {-h,  h, 0},  // Arriba-Izquierda
            { h,  h, 0},  // Arriba-Derecha
            { h, -h, 0},  // Abajo-Derecha
            {-h, -h, 0}   // Abajo-Izquierda
        };

        // Cambiamos a SOLVEPNP_ITERATIVE si hay mucha inclinación, 
        // ya que IPPE_SQUARE es excelente pero extremadamente sensible a ruidos de píxel en las esquinas.
        bool ok = cv::solvePnP(obj_pts, sorted_corners, K, dist,
                               result.rvec, result.tvec, false, cv::SOLVEPNP_ITERATIVE);
        
        result.valid = ok;
        return result;
    }

private:
    double qr_size_;
    zbar::ImageScanner scanner_;
};