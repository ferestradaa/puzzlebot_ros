#pragma once
#include <Eigen/Dense>
#include <unordered_map>

class ExtendedKalmanFilter{
    public:
        ExtendedKalmanFilter(
            const Eigen::Matrix3d & Q,
            const Eigen::Matrix2d & R,
            const std::unordered_map<int, Eigen::Vector2d> & landmark_map,
            const Eigen::Vector3d & initial_state = Eigen::Vector3d::Zero(),
            const Eigen::Matrix3d & initial_covariance = Eigen::Matrix3d::Identity())  

            : Q_(Q), R_(R), landmark_map_(landmark_map),
            mu_(initial_state),
            P_(initial_covariance)
            {}     


        //predict method inputs the control input: (twist message)
        void predict(double v, double omega, double dt){ //only lineal vel (x) is considered
            
        }
    private:
        Eigen::Vector3d mu_;
        Eigen::Matrix3d P_;
        Eigen::Matrix3d Q_;
        Eigen::Matrix2d R_;
        std::unordered_map<int, Eigen::Vector2d> landmark_map_;
    }; 