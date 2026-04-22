
/*
S sigma is covariance for EKF

F is jacobian for linearize the robots model [x, y, theta]
F = [1      0       -v*sin(theta + d_theta/2)]
    [0      1       v*cos(theta + d_theta/2)]
    [0      0       1                        ]

W is jacobian for input noise for EACH wheel (used for computing Q (kinmatics model confidence))
W = [[dt·R/2·cos(θ),   dt·R/2·cos(θ)],
     [dt·R/2·sin(θ),   dt·R/2·sin(θ)],
     [dt·R/L,          -dt·R/L       ]]

Both F and W uses the same model, but the partial derivative changes 

M is covariance of wheels noise
M = [[sigma²·|ωR|,     0         ],
     [    0   ,    sigma²·|ωL|  ]]

sigma squared is computed empiricamente so just change it depending on you

So Q = W @ K @ W.T

Now, covariance can be computed as 
S = F @ S @ F.T + Q
             
*/
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
        void predict(double vL, double vR, double rvR, double rvL, double dt){ 

            double theta_prev = mu_(2);
            double v = (vR + vL) / 2.0; //lineal pos of the robot
            double omega = (vR - vL) / L_; 
            double theta_mid = theta_prev + omega / 2.0; //mean of initial angle and new angle

            F_ << 1, 0, -v *std::sin(theta_mid) * dt, //jacobian of motion model
                 0, 1,  v * std::cos(theta_mid) * dt, 
                 0, 0,  1; 

            W_ << dt * r_ / 2.0 * std::cos(theta_mid), dt * r_ / 2.0 * std::cos(theta_mid),  //jacobian of input noise 
                  dt * r_ / 2.0 * std::sin(theta_mid), dt * r_ / 2.0 * std::sin(theta_mid), 
                  dt * r_ / L_,                    -dt * r_ / L_; 
            

            mu_(0) += v *std::cos(theta_mid) * dt; 
            mu_(1) += v * std::sin(theta_mid) * dt; 
            mu_(2) += omega * dt; 
            mu_(2) = std::atan2(std::sin(mu_(2)), std::cos(mu_(2)));


            M_ << sigma_squared_ * std::abs(rvR), 0.0,  //covariance of wheel noise
                 0.0,           sigma_squared_ * std::abs(rvL); 
        
    
            Q_ = W_ * M_ * W_.transpose(); // Noise of process

            P_ = F_ * P_ * F_.transpose() + Q_; //covarianza of state 
            
        }


        void update(const Eigen::Vector2d& landmark_world,   // known pos of landmark
                    const Eigen::Vector2d& z_detected){      // detected in robot frame

            //the node recieves arcuo pose in camera frame
            //decide to or not to use the orientation as input for filter, bc sould get a good cov that 
            //consideres yaw orientation (it is noise and probablu instroduces more noise
            //rather than improving the filter)

            double xL = landmark_world(0);
            double yL = landmark_world(1);
            double delta_x = xL - mu_(0); //xL is x detection of landmark same as yL
            double delta_y = yL - mu_(1); 


            Eigen::Vector2d h_mu;
            h_mu << std::cos(mu_(2)) * delta_x + std::sin(mu_(2)) * delta_y, // z_pred
                      - std::sin(mu_(2)) * delta_x + std::cos(mu_(2)) *delta_y; 

            Eigen::Vector2d y;
            y << z_detected(0) - h_mu(0), //inovation (xy landmark runtime detection - zpred)
                 z_detected(1) - h_mu(1); 
            
            Eigen::Matrix<double, 2, 3> H;
            H << -std::cos(mu_(2)),     -std::sin(mu_(2)),  -delta_x * std::sin(mu_(2)) + delta_y * std::cos(mu_(2)), //jacobian
                  std::sin(mu_(2)),     -std::cos(mu_(2)),  -delta_x * std::cos(mu_(2)) - delta_y * std::sin(mu_(2)); //change betwwen state and suposed state (aruco fixed pos)

            
            Eigen::Matrix2d S = H * P_ * H.transpose() + R_; //covariance of inovatation (combines covariance and sensor noise)

            Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse(); //kalman gain 

            mu_ += K * y; //update state
            mu_(2) = std::atan2(std::sin(mu_(2)), std::cos(mu_(2))); //normalize angle 

            //P_ = (Eigen::Matrix3d::Identity() - K * H) * P_; //update state covariance

            Eigen::Matrix3d I_KH = Eigen::Matrix3d::Identity() - K * H;
            P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();
                 
        }

        Eigen::Vector3d getState()      const { return mu_; }
        Eigen::Matrix3d getCovariance() const { return P_;  }


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