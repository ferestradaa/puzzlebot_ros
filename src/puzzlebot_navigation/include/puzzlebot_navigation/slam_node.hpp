#pragma once

#include "puzzlebot_navigation/likelihood_field.hpp"
#include "puzzlebot_navigation/graph_slam.hpp"
#include "puzzlebot_navigation/map_manager.hpp"
#include "puzzlebot_navigation/motion_model.hpp"
#include "puzzlebot_navigation/occupancy_grid_map.hpp"
#include "puzzlebot_navigation/particle_filter_cuda.hpp"
#include "puzzlebot_navigation/scan_processor.hpp"
#include "puzzlebot_navigation/slam_state_machine.hpp"
#include "puzzlebot_navigation/slam_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>

#include <filesystem>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace puzzlebot_navigation
{

class SLAMNode : public rclcpp::Node
{
public:
  SLAMNode();
  ~SLAMNode() override;

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::LaserScan,
    nav_msgs::msg::Odometry>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  void declare_and_load_parameters();
  std::string find_workspace_maps_directory() const;

  void synced_callback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);

  void run_slam_cycle();

  void initialize_particles_hybrid_known_map(float near_ratio);
  void publish_map(bool force = false);
  void publish_particles(bool force = false);
  void publish_map_to_odom_tf(const Pose2D & map_base, const Pose2D & odom_base);
  bool should_publish_map_now() const;
  void save_map_to_pgm_yaml(const std::string & prefix);
  void log_state_change_if_needed(SLAMState previous_state, SLAMState new_state);
  void log_large_tracking_risk_change_if_needed();
  void publish_graph_markers();

  // General config
  std::string map_frame_{"map"};
  std::string map_topic_{"/slam_map"};
  std::string particles_topic_{"/slam_particles"};

  float x_min_{-6.0f};
  float x_max_{6.0f};
  float y_min_{-6.0f};
  float y_max_{6.0f};
  float res_{0.05f};
  float l_occ_{1.00f};
  float l_free_{-0.1f};
  float l_min_{-6.0f};
  float l_max_{12.0f};
  float l_occ_lock_{11.5f};
  float l_free_lock_{-5.5f};

  int scan_step_{8};
  float usable_max_range_{3.0f};
  float hit_range_margin_{0.05f};
  float scan_angle_offset_{kPi * 0.5f};

  int N_{1000};
  float initial_x_{0.0f};
  float initial_y_{0.0f};
  float initial_theta_{0.0f};
  float initial_std_xy_{0.08f};
  float initial_std_theta_{0.10f};

  float motion_noise_rot1_{0.03f};
  float motion_noise_trans_{0.03f};
  float resample_pos_noise_{0.015f};
  float resample_theta_noise_{0.02f};

  int bootstrap_min_scans_{6};
  int bootstrap_map_period_{2};
  int bootstrap_min_mapped_scans_{3};
  int bootstrap_validation_limit_{4};
  int bad_conf_limit_{8};
  float neff_lost_ratio_{0.15f};
  float neff_recovered_ratio_{0.45f};
  float map_update_neff_ratio_{0.50f};
  float resample_neff_ratio_{0.65f};

  int min_valid_scan_points_{50};
  float motion_alpha_rot_from_rot_{0.08f};
  float motion_alpha_rot_from_trans_{0.03f};
  float motion_alpha_trans_from_trans_{0.08f};
  float motion_alpha_trans_from_rot_{0.03f};
  float odom_cov_trans_scale_{1.0f};
  float odom_cov_rot_scale_{1.0f};
  float motion_noise_min_{0.005f};
  float motion_noise_max_{0.35f};

  float likelihood_sigma_{0.05f};
  float likelihood_max_distance_{0.25f};
  int likelihood_rebuild_period_{5};
  int likelihood_min_dirty_occupied_cells_{20};
  float min_good_score_{-2.0f};

  int min_known_free_cells_for_reset_{200};
  int map_publish_min_period_ms_{700};
  bool publish_particles_debug_{true};
  int particles_publish_period_{5};
  int particles_publish_counter_{0};
  int particles_publish_stride_{4};
  int max_scan_points_gpu_{2048};
  float full_map_upload_dirty_ratio_{0.10f};

  bool loop_closure_enabled_{true};
  int loop_min_mapped_scans_{18};
  int loop_min_keyframe_gap_{10};
  int loop_cooldown_cycles_{10};
  int max_keyframes_{300};
  float keyframe_min_translation_{0.35f};
  float keyframe_min_rotation_{0.35f};
  float loop_search_radius_{0.50f};
  float loop_max_angle_diff_{1.80f};
  float loop_min_scan_score_{0.55f};
  bool graph_visualization_enabled_{true};

  std::string map_save_dir_;
  std::string map_save_prefix_{"map"};

  float last_logged_tracking_risk_{-1.0f};
  float tracking_risk_log_delta_{25.0f};
  float tracking_risk_warning_percent_{75.0f};

  // ROS
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<Sync> sync_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_;
  bool new_synced_pair_{false};

  // Modules
  OccupancyGridMap map_;
  MapManager map_manager_{map_};
  LikelihoodField likelihood_field_;
  ScanProcessor scan_processor_;
  MotionModel motion_model_;
  ParticleFilterCUDA particle_filter_;
  SLAMStateMachine state_machine_;
  GraphSLAM graph_slam_;

  // Runtime state
  std::mt19937 rng_{std::random_device{}()};
  std::optional<Pose2D> curr_odom_pose_local_;
  std::optional<Pose2D> prev_odom_pose_local_;
  bool odom_origin_captured_{false};
  std::optional<Pose2D> estimated_pose_mcl_;
  std::optional<Pose2D> corrected_pose_;
  int total_valid_scan_counter_{0};
  rclcpp::Time last_map_publish_time_{0, 0, RCL_ROS_TIME};

  std::vector<float> pub_px_buffer_;
  std::vector<float> pub_py_buffer_;
  std::vector<float> pub_ptheta_buffer_;
};

}  // namespace puzzlebot_navigation