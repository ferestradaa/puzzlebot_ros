#include <optional>
#include <cmath>

struct Pose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

struct PoseValidationResult {
    int count;
    std::optional<Pose> final_pose;
};

class PoseValidator {
public:
    PoseValidator(double pos_threshold, double yaw_threshold, int setpoint)
        : pos_threshold_(pos_threshold),
          yaw_threshold_(yaw_threshold),
          setpoint_(setpoint),
          count_(0),
          has_previous_(false)
    {}

    PoseValidationResult validate(const Pose& current) {
        if (!has_previous_) {
            previous_ = current;
            has_previous_ = true;
            return {count_, std::nullopt};
        }

        double dx = current.x - previous_.x;
        double dy = current.y - previous_.y;
        double dz = current.z - previous_.z;
        double pos_error = std::sqrt(dx*dx + dy*dy + dz*dz);

        double yaw_error = current.yaw - previous_.yaw;
        // normalixe yaw angle
        yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));
        yaw_error = std::abs(yaw_error);

        if (pos_error <= pos_threshold_ && yaw_error <= yaw_threshold_) {
            count_++;
        } else {
            count_ = 0;
        }

        previous_ = current;

        if (count_ >= setpoint_) {
            return {count_, current};
        }

        return {count_, std::nullopt};
    }

    void reset() {
        count_ = 0;
        has_previous_ = false;
    }

private:
    double pos_threshold_;
    double yaw_threshold_;
    int setpoint_;
    int count_;
    bool has_previous_;
    Pose previous_;
};