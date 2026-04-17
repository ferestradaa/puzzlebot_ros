/*
This node generates an occumapy grid at 5 HZ 
The map is updated at runtime for approachinng complete SLAM behaviour

Input: 
1. LIDAR scan using sensor frame 
2. Robot pose (odometry as EKF output) in map frame TENTATIVO

Output: 
1. log odds matrix as nav_msgs/OccupancyGrid
This grid is consumed by the path planner to evaluate which cells can be used for
motion

*/