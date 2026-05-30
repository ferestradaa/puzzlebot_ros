#pragma once
#include <Eigen/Dense>
#include <unordered_map>
#include <cmath>

class ExtendedKalmanFilter{
    public:
        ExtendedKalmanFilter(
            const double r, double L_, 
            const Eigen::Vector3d & initial_state = Eigen::Vector3d::Zero(), //zero so pos and orient are lit 0,0,0
            const Eigen::Matrix3d & initial_covariance = Eigen::Matrix3d::Identity(),
            const Eigen::Matrix2d & initial_R = Eigen::Matrix2d::Identity() * 0.01)

            : P_(initial_covariance),
            mu_(initial_state),
            R_(initial_R),
            r_(r),
            L_(L_)
            {}

        //predict method inputs the control input: (twist message)
        void predict(double vL, double vR, double rvL, double rvR, double dt){ 

            double theta_prev = mu_(2);
            double v = (vR + vL) / 2.0; //lineal vel of the robot
            double omega = (vR - vL) / L_; 
            double theta_mid = theta_prev + omega * dt / 2.0; //mean of initial angle and new angle

            F_ << 1, 0, -v *std::sin(theta_mid) * dt, //jacobian of motion model
                 0, 1,  v * std::cos(theta_mid) * dt, 
                 0, 0,  1; 

            W_ << dt * r_ / 2.0 * std::cos(theta_mid), dt * r_ / 2.0 * std::cos(theta_mid),  //jacobian of input noise 
                  dt * r_ / 2.0 * std::sin(theta_mid), dt * r_ / 2.0 * std::sin(theta_mid), //of wheels
                  dt * r_ / L_,                    -dt * r_ / L_; 
            

            mu_(0) += v *std::cos(theta_mid) * dt; 
            mu_(1) += v * std::sin(theta_mid) * dt; 
            mu_(2) += omega * dt; 
            mu_(2) = std::atan2(std::sin(mu_(2)), std::cos(mu_(2)));


            M_ << sigma_squared_ * std::abs(rvR), 0.0,  //covariance of wheel noise
                 0.0,           sigma_squared_ * std::abs(rvL); 
        
    
            Q_ = W_ * M_ * W_.transpose(); // Noise of process

            P_ = F_ * P_ * F_.transpose() + Q_; //covarianza of state: uses the jacobian of model and initial covariance 
            
        }


        // wraps angle to [-pi, pi]
        static inline double wrap(double a){ return std::atan2(std::sin(a), std::cos(a)); }

        // landmark_world: x, y, theta del tag en el mundo (del mapa)
        // z_xy: x, y del tag en frame robot
        // yaw_rel: yaw del tag relativo al robot (ruidoso)
        void update(const Eigen::Vector3d& landmark_world,
                    const Eigen::Vector2d& z_xy,
                    double yaw_rel)
        {
            double xL = landmark_world(0);
            double yL = landmark_world(1);
            double thetaL = landmark_world(2);
            double dx = xL - mu_(0);
            double dy = yL - mu_(1);
            double d  = std::hypot(z_xy(0), z_xy(1));
            double c = std::cos(mu_(2));
            double s = std::sin(mu_(2));
            // prediccion de la posicion del tag en frame robot
            double hx =  c * dx + s * dy;
            double hy = -s * dx + c * dy;
            double r_xy = sigma_base_sq + alpha * d * d;
            // chequeo de flip: el yaw medido implica una orientacion de tag en el mundo
            // que debe coincidir con la conocida. si no, la deteccion esta flippeada
            double yaw_world_meas = wrap(mu_(2) + yaw_rel);  // ajustar signo segun tu detector
            double dot = std::cos(thetaL) * std::cos(yaw_world_meas)
                       + std::sin(thetaL) * std::sin(yaw_world_meas);
            bool use_yaw = (dot > 0.7);  // discrepancia < ~45 grados
            if (use_yaw) {
                // update 3D: posicion + yaw
                Eigen::Vector3d h;
                h << hx, hy, wrap(thetaL - mu_(2));  // mismo signo que el chequeo de arriba
                Eigen::Vector3d z;
                z << z_xy(0), z_xy(1), yaw_rel;
                Eigen::Vector3d y;
                y(0) = z(0) - h(0);
                y(1) = z(1) - h(1);
                y(2) = wrap(z(2) - h(2));
                Eigen::Matrix3d H;
                H << -c, -s, -dx*s + dy*c,
                      s, -c, -dx*c - dy*s,
                      0,  0, -1.0;  // revisar signo de esta fila junto con h(2)
                Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
                R(0,0) = r_xy;
                R(1,1) = r_xy;
                R(2,2) = sigma_yaw_sq;  // grande a proposito: 0.05 - 0.1 rad^2
                Eigen::Matrix3d S = H * P_ * H.transpose() + R;
                double mahal = y.dot(S.inverse() * y);
                if (mahal > 7.81) return;  // chi2 95% con 3 DoF
                Eigen::Matrix3d K = P_ * H.transpose() * S.inverse();
                mu_ += K * y;
                mu_(2) = wrap(mu_(2));
                Eigen::Matrix3d I_KH = Eigen::Matrix3d::Identity() - K * H;
                P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
            } else {
                // update 2D: solo posicion (yaw no confiable este frame)
                Eigen::Vector2d h(hx, hy);
                Eigen::Vector2d y = z_xy - h;
                Eigen::Matrix<double,2,3> H;
                H << -c, -s, -dx*s + dy*c,
                      s, -c, -dx*c - dy*s;
                Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * r_xy;
                Eigen::Matrix2d S = H * P_ * H.transpose() + R;
                double mahal = y.dot(S.inverse() * y);
                if (mahal > 5.99) return;  // chi2 95% con 2 DoF
                Eigen::Matrix<double,3,2> K = P_ * H.transpose() * S.inverse();
                mu_ += K * y;
                mu_(2) = wrap(mu_(2));
                Eigen::Matrix3d I_KH = Eigen::Matrix3d::Identity() - K * H;
                P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
            }
        }

        Eigen::Vector3d getState()      const { return mu_; }
        Eigen::Matrix3d getCovariance() const { return P_;  }
        double sigma_base_sq = 0.0025;  // 5cm std a 1m
        double alpha = 0.01;
        double sigma_yaw_sq = 0.05;  // tunable: start high, tighten if yaw is reliable


    private:
        Eigen::Vector3d mu_; //robot state [x, y, theta]
        Eigen::Matrix3d Q_; // Noise of process
        Eigen::Matrix3d F_; //jacobian of motion model
        Eigen::Matrix2d M_; //covariance of wheel noise
        Eigen::Matrix3d P_; //covariance of state
        Eigen::Matrix2d R_;
        Eigen::Matrix<double,3,2> W_; 
        double sigma_squared_ = 0.1;
        double r_, L_; 
   

    };