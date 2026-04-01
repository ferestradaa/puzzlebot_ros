import math 
from typing import Tuple

def twist_to_wheel_velocities(linear_x, angular_z, wheel_radius, wheel_base) -> Tuple[float, float]:
    v_left = (linear_x - angular_z *wheel_base / 2.0) / wheel_radius
    v_right = (linear_x + angular_z * wheel_base / 2.0) / wheel_radius
    return v_left, v_right

def wheel_velocites_to_twist(left_rad, right_rad, wheel_radius, wheel_base) -> Tuple[float, float]:
    linear_x = wheel_radius * (left_rad + right_rad) / 2.0
    angular_z = wheel_radius * (right_rad -left_rad) / wheel_base
    return linear_x, angular_z


def integrate_odometry(x, y, theta, linear_x, angular_z, dt):
    theta_new = theta + angular_z * dt
    x_new = x + linear_x * math.cos(theta + angular_z * dt / 2.0) * dt
    y_new = y + linear_x * math.sin(angular_z * dt / 2.0) * dt
    return x_new, y_new, theta_new

def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:

    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)

def rad_to_deg(rad: float) -> float:
    return rad * 57.2958


def deg_to_rad(deg: float) -> float:
    return deg / 57.2958


