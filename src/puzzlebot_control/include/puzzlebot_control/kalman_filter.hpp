#pragma once
#include <Eigen/Dense>
#include <unordered_map>
#include <cmath>
class ExtendedKalmanFilter{
    public:
        // diagnostics returned by update() so the node can log them.
        // The EKF stays ROS-agnostic (no logger dependency here).
        struct UpdateInfo {
            bool   accepted         = false;  // passed the gate OR was relocalized
            bool   used_yaw         = false;  // true = 3D update, false = 2D (yaw flip rejected)
            bool   relocalized      = false;  // NEW: hard pose reset happened this call
            double mahalanobis      = 0.0;    // gating distance for this detection
            double yaw_world_meas   = 0.0;    // tag world yaw implied by the measurement
            double yaw_world_map    = 0.0;    // tag world yaw from the map (reference)
            double yaw_rel_raw      = 0.0;    // NEW: raw yaw_rel measurement (for offset calibration)
            double yaw_rel_expected = 0.0;    // NEW: yaw_rel that the current pose predicts
        };

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
           
            // piso de ruido de proceso: evita que P colapse entre detecciones
            P_(0,0) += pos_density_   * dt;
            P_(1,1) += pos_density_   * dt;
            P_(2,2) += theta_density_ * dt;
            // forzar simetria, evita deriva numerica
            P_ = 0.5 * (P_ + P_.transpose());
        }
        // wraps angle to [-pi, pi]
        static inline double wrap(double a){ return std::atan2(std::sin(a), std::cos(a)); }
        // landmark_world: x, y, theta del tag en el mundo (del mapa)
        // z_xy: x, y del tag en frame robot
        // yaw_rel: yaw del tag relativo al robot (ruidoso)
        UpdateInfo update(const Eigen::Vector3d& landmark_world,
                    const Eigen::Vector2d& z_xy,
                    double yaw_rel, 
                    double w_robot = 0.0)
        {
            UpdateInfo info;
            info.yaw_world_map = landmark_world(2);

            double xL = landmark_world(0);
            double yL = landmark_world(1);
            double thetaL = landmark_world(2);

            // NEW: apply the fixed detector->map yaw offset BEFORE using yaw_rel.
            // yaw_offset is the constant rotation between what your detector reports
            // and the map heading convention. Calibrate it from the node logs
            // (yaw_rel_raw vs yaw_rel_expected at a known static pose).
            double yaw_rel_c = wrap(yaw_rel - yaw_offset);
            info.yaw_rel_raw      = yaw_rel;
            info.yaw_rel_expected = wrap(thetaL - mu_(2));  // what yaw_rel should be at this pose


            if (!localized_) {
                double th = wrap(thetaL - yaw_rel_c);
                double cr = std::cos(th), sr = std::sin(th);
                mu_(0) = landmark_world(0) - (cr * z_xy(0) - sr * z_xy(1));
                mu_(1) = landmark_world(1) - (sr * z_xy(0) + cr * z_xy(1));
                mu_(2) = th;

                // covarianza moderada: confiamos en el tag pero dejamos margen
                // para que las siguientes detecciones refinen
                P_ = Eigen::Matrix3d::Zero();
                P_(0,0) = 0.05;
                P_(1,1) = 0.05;
                P_(2,2) = 0.02;

                localized_ = true;
                info.accepted    = true;
                info.relocalized = true;
                info.used_yaw    = true;
                info.yaw_world_meas = wrap(mu_(2) + yaw_rel_c);
                return info;
}

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
            double yaw_world_meas = wrap(mu_(2) + yaw_rel_c);  // NEW: uses corrected yaw
            info.yaw_world_meas = yaw_world_meas;
            double dot = std::cos(thetaL) * std::cos(yaw_world_meas)
                       + std::sin(thetaL) * std::sin(yaw_world_meas);
            bool use_yaw = just_initialized_ ? true : (dot > 0.7);
            just_initialized_ = false; // consume the flag
            info.used_yaw = use_yaw;
            if (use_yaw) {
                // update 3D: posicion + yaw
                Eigen::Vector3d h;
                h << hx, hy, wrap(thetaL - mu_(2));  // mismo signo que el chequeo de arriba
                Eigen::Vector3d z;
                z << z_xy(0), z_xy(1), yaw_rel_c;    // NEW: corrected yaw, not raw
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
                // yaw_var derived from sigma_yaw_sq, inflated while turning fast.
                double yaw_var = sigma_yaw_sq;
                if (std::abs(w_robot) > 0.5) yaw_var *= 4.0;
                R(2,2) = yaw_var; 
                Eigen::Matrix3d S = H * P_ * H.transpose() + R;
                double mahal = y.dot(S.inverse() * y);
                info.mahalanobis = mahal;
                if (mahal > 7.81) {  // chi2 95% con 3 DoF
                    // NEW: rejected. count it and relocalize if stuck.
                    info.relocalized = maybeRelocalize(landmark_world, z_xy, yaw_rel_c, info);
                    return info;
                }
                Eigen::Matrix3d K = P_ * H.transpose() * S.inverse();
                mu_ += K * y;
                mu_(2) = wrap(mu_(2));
                Eigen::Matrix3d I_KH = Eigen::Matrix3d::Identity() - K * H;
                P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
                P_ = 0.5 * (P_ + P_.transpose());  // keep symmetry after Joseph update
                info.accepted = true;
                consecutive_rejections_ = 0;       // NEW: good update, reset reloc counter
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
                info.mahalanobis = mahal;
                if (mahal > 5.99) {  // chi2 95% con 2 DoF
                    // NEW: rejected. count it and relocalize if stuck.
                    info.relocalized = maybeRelocalize(landmark_world, z_xy, yaw_rel_c, info);
                    return info;
                }
                Eigen::Matrix<double,3,2> K = P_ * H.transpose() * S.inverse();
                mu_ += K * y;
                mu_(2) = wrap(mu_(2));
                Eigen::Matrix3d I_KH = Eigen::Matrix3d::Identity() - K * H;
                P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
                P_ = 0.5 * (P_ + P_.transpose());  // keep symmetry after Joseph update
                info.accepted = true;
                consecutive_rejections_ = 0;       // NEW
            }
            return info;
        }
        Eigen::Vector3d getState()      const { return mu_; }
        Eigen::Matrix3d getCovariance() const { return P_;  }

        void setState(const Eigen::Vector3d& s) {
            mu_ = s;
            // post-init covariance: confident but not zero
            P_ = Eigen::Matrix3d::Zero();
            P_(0,0) = 0.1;
            P_(1,1) = 0.1;
            P_(2,2) = 0.05;
            just_initialized_ = true;
        }


        
        double sigma_base_sq = 0.0025;  // 5cm std a 1m
        double alpha = 0.01;
        double sigma_yaw_sq = 0.05;  // tunable: start high, tighten if yaw is reliable
        // NEW: tunables for the yaw offset and the relocalization mode
        double yaw_offset = M_PI / 2.0;  // CALIBRATE from logs; set to the stable offset you read
        int    reloc_after = 5;          // consecutive rejected detections before a hard reset
        double reloc_cov   = 1.0;        // P diagonal after relocalizing (1.0 -> std 1m / 1rad)


    private:
        // NEW: hard pose reset from a single tag when the filter is stuck rejecting.
        // A full relative-pose observation makes the global robot pose directly solvable:
        //   p_tag_world = p_robot + R(theta) * p_tag_in_robot
        // so p_robot = p_tag - R(theta) * z_xy, with theta = thetaL - yaw_rel_c.
        // NOTE: this trusts yaw_offset, so calibrate yaw first or the reset heading is wrong.
        bool maybeRelocalize(const Eigen::Vector3d& landmark_world,
                             const Eigen::Vector2d& z_xy,
                             double yaw_rel_c, UpdateInfo& info)
        {
            consecutive_rejections_++;
            if (consecutive_rejections_ < reloc_after) return false;

            double xL = landmark_world(0);
            double yL = landmark_world(1);
            double thetaL = landmark_world(2);

            double th = wrap(thetaL - yaw_rel_c);   // robot heading implied by the tag
            double cr = std::cos(th), sr = std::sin(th);
            mu_(0) = xL - (cr * z_xy(0) - sr * z_xy(1));
            mu_(1) = yL - (sr * z_xy(0) + cr * z_xy(1));
            mu_(2) = th;

            P_ = Eigen::Matrix3d::Identity() * reloc_cov;  // inflate so next detections refine
            consecutive_rejections_ = 0;
            info.accepted = true;   // treat the reset as a valid fix for the node
            return true;
        }

        Eigen::Vector3d mu_; //robot state [x, y, theta]
        Eigen::Matrix3d Q_; // Noise of process
        Eigen::Matrix3d F_; //jacobian of motion model
        Eigen::Matrix2d M_; //covariance of wheel noise
        Eigen::Matrix3d P_; //covariance of state
        Eigen::Matrix2d R_; // NOTE: currently unused; update() builds R locally per detection
        Eigen::Matrix<double,3,2> W_; 
        double sigma_squared_ = 0.1;
        double r_, L_; 
        double pos_density_   = 0.01;  // m^2/s,   piso de incertidumbre de posicion
        double theta_density_ = 0.05;  // rad^2/s, piso de incertidumbre de yaw 
        int consecutive_rejections_ = 0;  // NEW: drives the relocalization trigger
        bool just_initialized_ = false;
        bool localized_ = false;
   
    };