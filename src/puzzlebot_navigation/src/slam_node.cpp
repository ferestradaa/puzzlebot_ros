#include "puzzlebot_navigation/slam_node.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <stdexcept>

using std::placeholders::_1;
using std::placeholders::_2;

namespace puzzlebot_navigation
{

SLAMNode::SLAMNode()
: Node("slam_node")
{
  declare_and_load_parameters();

  if (map_save_dir_.empty()) {
    map_save_dir_ = find_workspace_maps_directory();
  }
  std::filesystem::create_directories(map_save_dir_);

  RCLCPP_INFO(
    get_logger(),
    "Map output directory: %s | prefix: %s",
    map_save_dir_.c_str(),
    map_save_prefix_.c_str());

  map_.configure(
    x_min_, x_max_, y_min_, y_max_, res_,
    l_occ_, l_free_, l_min_, l_max_,
    l_occ_lock_, l_free_lock_);
  map_.initialize();

  map_manager_.configure(3);

  scan_processor_.configure(
    scan_step_, max_scan_points_gpu_, min_valid_scan_points_,
    usable_max_range_, hit_range_margin_, scan_angle_offset_);

  motion_model_.configure(
    motion_noise_rot1_, motion_noise_trans_,
    motion_alpha_rot_from_rot_, motion_alpha_rot_from_trans_,
    motion_alpha_trans_from_trans_, motion_alpha_trans_from_rot_,
    odom_cov_trans_scale_, odom_cov_rot_scale_,
    motion_noise_min_, motion_noise_max_);

  likelihood_field_.configure(
    likelihood_sigma_, likelihood_max_distance_,
    likelihood_rebuild_period_, likelihood_min_dirty_occupied_cells_);
  likelihood_field_.initialize(map_);

  particle_filter_.configure(
    N_, max_scan_points_gpu_, resample_pos_noise_, resample_theta_noise_);
  particle_filter_.initialize(map_.size());
  particle_filter_.initialize_particles_near(
    initial_x_, initial_y_, initial_theta_, initial_std_xy_, initial_std_theta_, rng_);
  particle_filter_.upload_particles();
  particle_filter_.upload_initial_map(map_);
  particle_filter_.upload_likelihood_field(likelihood_field_);

  state_machine_.configure(
    bootstrap_min_scans_, bootstrap_min_mapped_scans_, bootstrap_validation_limit_, bootstrap_map_period_,
    bad_conf_limit_, min_good_score_, neff_lost_ratio_, neff_recovered_ratio_,
    map_update_neff_ratio_, resample_neff_ratio_, N_);

  graph_slam_.configure(
    loop_closure_enabled_, loop_min_mapped_scans_, loop_min_keyframe_gap_, loop_cooldown_cycles_,
    max_keyframes_, keyframe_min_translation_, keyframe_min_rotation_, loop_search_radius_,
    loop_max_angle_diff_, loop_min_scan_score_);

  scan_sub_.subscribe(this, "/scan");
  odom_sub_.subscribe(this, "/odom");
  sync_ = std::make_shared<Sync>(SyncPolicy(20), scan_sub_, odom_sub_);
  sync_->registerCallback(std::bind(&SLAMNode::synced_callback, this, _1, _2));

  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, 10);
  particles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(particles_topic_, 10);
  graph_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/slam_graph", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_px_buffer_.reserve(static_cast<size_t>((N_ + particles_publish_stride_ - 1) / particles_publish_stride_));
  pub_py_buffer_.reserve(static_cast<size_t>((N_ + particles_publish_stride_ - 1) / particles_publish_stride_));
  pub_ptheta_buffer_.reserve(static_cast<size_t>((N_ + particles_publish_stride_ - 1) / particles_publish_stride_));

  last_map_publish_time_ = now();

  timer_ = create_wall_timer(
    std::chrono::milliseconds(250),
    std::bind(&SLAMNode::run_slam_cycle, this));

  publish_map(true);
  publish_particles(true);

  RCLCPP_INFO(
    get_logger(),
    "slam_node started. Listening to /scan and /odom | Publishing %s and %s",
    map_topic_.c_str(), particles_topic_.c_str());
}

SLAMNode::~SLAMNode()
{
  try {
    save_map_to_pgm_yaml(map_save_prefix_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Could not save map while destroying node: %s", e.what());
  }
}

void SLAMNode::declare_and_load_parameters()
{
  // =========================
  // Output / save
  // =========================
  declare_parameter<std::string>("map_save_dir", "");
  declare_parameter<std::string>("map_save_prefix", "map");

  // =========================
  // Frames / topics
  // =========================
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("map_topic", "/slam_map");
  declare_parameter<std::string>("particles_topic", "/slam_particles");

  // =========================
  // Initial pose
  // =========================
  declare_parameter<double>("initial_x", 1.0);
  declare_parameter<double>("initial_y", 0.5);
  declare_parameter<double>("initial_theta", 0.0);
  declare_parameter<double>("initial_std_xy", 0.08);
  declare_parameter<double>("initial_std_theta", 0.10);

  // =========================
  // Map geometry
  // =========================
  declare_parameter<double>("x_min", -6.0);
  declare_parameter<double>("x_max", 6.0);
  declare_parameter<double>("y_min", -6.0);
  declare_parameter<double>("y_max", 6.0);
  declare_parameter<double>("resolution", 0.05);

  // =========================
  // Occupancy log-odds
  // =========================
  declare_parameter<double>("l_occ", 1.00);
  declare_parameter<double>("l_free", -0.10);
  declare_parameter<double>("l_min", -6.0);
  declare_parameter<double>("l_max", 12.0);
  declare_parameter<double>("l_occ_lock", 11.5);
  declare_parameter<double>("l_free_lock", -5.5);

  // =========================
  // Scan processing
  // =========================
  declare_parameter<int>("scan_step", 8);
  declare_parameter<int>("max_scan_points_gpu", 2048);
  declare_parameter<int>("min_valid_scan_points", 50);
  declare_parameter<double>("usable_max_range", 3.5);
  declare_parameter<double>("hit_range_margin", 0.05);
  declare_parameter<double>("scan_angle_offset", static_cast<double>(kPi * 0.5f));

  // =========================
  // Particle filter
  // =========================
  declare_parameter<int>("num_particles", 1024);
  declare_parameter<double>("resample_pos_noise", 0.03);
  declare_parameter<double>("resample_theta_noise", 0.03);
  declare_parameter<double>("full_map_upload_dirty_ratio", 0.10);

  // =========================
  // Motion model
  // =========================
  declare_parameter<double>("motion_noise_rot1", 0.03);
  declare_parameter<double>("motion_noise_trans", 0.03);

  declare_parameter<double>("motion_alpha_rot_from_rot", 0.08);
  declare_parameter<double>("motion_alpha_rot_from_trans", 0.03);
  declare_parameter<double>("motion_alpha_trans_from_trans", 0.08);
  declare_parameter<double>("motion_alpha_trans_from_rot", 0.03);

  declare_parameter<double>("odom_cov_trans_scale", 1.0);
  declare_parameter<double>("odom_cov_rot_scale", 1.0);
  declare_parameter<double>("motion_noise_min", 0.005);
  declare_parameter<double>("motion_noise_max", 0.35);

  // =========================
  // Likelihood field
  // =========================
  declare_parameter<double>("likelihood_sigma", 0.15);
  declare_parameter<double>("likelihood_max_distance", 0.60);
  declare_parameter<int>("likelihood_rebuild_period", 5);
  declare_parameter<int>("likelihood_min_dirty_occupied_cells", 20);

  // =========================
  // SLAM state machine
  // =========================
  declare_parameter<int>("bootstrap_min_scans", 6);
  declare_parameter<int>("bootstrap_map_period", 2);
  declare_parameter<int>("bootstrap_min_mapped_scans", 3);
  declare_parameter<int>("bootstrap_validation_scans", 4);

  declare_parameter<int>("bad_conf_limit", 8);
  declare_parameter<double>("min_good_score", -2.0);

  declare_parameter<double>("neff_lost_ratio", 0.15);
  declare_parameter<double>("neff_recovered_ratio", 0.45);
  declare_parameter<double>("map_update_neff_ratio", 0.50);
  declare_parameter<double>("resample_neff_ratio", 0.65);

  // =========================
  // Graph SLAM / loop closure
  // =========================
  declare_parameter<bool>("loop_closure_enabled", true);
  declare_parameter<int>("loop_min_mapped_scans", 18);
  declare_parameter<int>("loop_min_keyframe_gap", 10);
  declare_parameter<int>("loop_cooldown_cycles", 10);
  declare_parameter<int>("max_keyframes", 300);
  declare_parameter<double>("keyframe_min_translation", 0.35);
  declare_parameter<double>("keyframe_min_rotation", 0.35);
  declare_parameter<double>("loop_search_radius", 0.5);
  declare_parameter<double>("loop_max_angle_diff", 1.80);
  declare_parameter<double>("loop_min_scan_score", 0.55);

  // =========================
  // Recovery / reset
  // =========================
  declare_parameter<int>("min_known_free_cells_for_reset", 200);

  // =========================
  // Publishing / debug
  // =========================
  declare_parameter<bool>("publish_particles_debug", true);
  declare_parameter<int>("particles_publish_period", 5);
  declare_parameter<int>("particles_publish_stride", 4);
  declare_parameter<int>("map_publish_min_period_ms", 750);

  declare_parameter<double>("tracking_risk_log_delta", 25.0);
  declare_parameter<double>("tracking_risk_warning_percent", 75.0);
  declare_parameter<bool>("graph_visualization_enabled", true);

  // =========================
  // Load params
  // =========================
  map_save_dir_ = get_parameter("map_save_dir").as_string();
  map_save_prefix_ = get_parameter("map_save_prefix").as_string();

  map_frame_ = get_parameter("map_frame").as_string();
  map_topic_ = get_parameter("map_topic").as_string();
  particles_topic_ = get_parameter("particles_topic").as_string();

  initial_x_ = static_cast<float>(get_parameter("initial_x").as_double());
  initial_y_ = static_cast<float>(get_parameter("initial_y").as_double());
  initial_theta_ = static_cast<float>(get_parameter("initial_theta").as_double());
  initial_std_xy_ = static_cast<float>(get_parameter("initial_std_xy").as_double());
  initial_std_theta_ = static_cast<float>(get_parameter("initial_std_theta").as_double());

  x_min_ = static_cast<float>(get_parameter("x_min").as_double());
  x_max_ = static_cast<float>(get_parameter("x_max").as_double());
  y_min_ = static_cast<float>(get_parameter("y_min").as_double());
  y_max_ = static_cast<float>(get_parameter("y_max").as_double());
  res_ = static_cast<float>(get_parameter("resolution").as_double());

  l_occ_ = static_cast<float>(get_parameter("l_occ").as_double());
  l_free_ = static_cast<float>(get_parameter("l_free").as_double());
  l_min_ = static_cast<float>(get_parameter("l_min").as_double());
  l_max_ = static_cast<float>(get_parameter("l_max").as_double());
  l_occ_lock_ = static_cast<float>(get_parameter("l_occ_lock").as_double());
  l_free_lock_ = static_cast<float>(get_parameter("l_free_lock").as_double());

  scan_step_ = std::max(1, static_cast<int>(get_parameter("scan_step").as_int()));
  max_scan_points_gpu_ = std::max(1, static_cast<int>(get_parameter("max_scan_points_gpu").as_int()));
  min_valid_scan_points_ = std::max(1, static_cast<int>(get_parameter("min_valid_scan_points").as_int()));
  usable_max_range_ = static_cast<float>(get_parameter("usable_max_range").as_double());
  hit_range_margin_ = static_cast<float>(get_parameter("hit_range_margin").as_double());
  scan_angle_offset_ = static_cast<float>(get_parameter("scan_angle_offset").as_double());

  N_ = std::max(1, static_cast<int>(get_parameter("num_particles").as_int()));
  resample_pos_noise_ = static_cast<float>(get_parameter("resample_pos_noise").as_double());
  resample_theta_noise_ = static_cast<float>(get_parameter("resample_theta_noise").as_double());
  full_map_upload_dirty_ratio_ =
    static_cast<float>(get_parameter("full_map_upload_dirty_ratio").as_double());

  motion_noise_rot1_ = static_cast<float>(get_parameter("motion_noise_rot1").as_double());
  motion_noise_trans_ = static_cast<float>(get_parameter("motion_noise_trans").as_double());

  motion_alpha_rot_from_rot_ =
    static_cast<float>(get_parameter("motion_alpha_rot_from_rot").as_double());
  motion_alpha_rot_from_trans_ =
    static_cast<float>(get_parameter("motion_alpha_rot_from_trans").as_double());
  motion_alpha_trans_from_trans_ =
    static_cast<float>(get_parameter("motion_alpha_trans_from_trans").as_double());
  motion_alpha_trans_from_rot_ =
    static_cast<float>(get_parameter("motion_alpha_trans_from_rot").as_double());

  odom_cov_trans_scale_ = static_cast<float>(get_parameter("odom_cov_trans_scale").as_double());
  odom_cov_rot_scale_ = static_cast<float>(get_parameter("odom_cov_rot_scale").as_double());
  motion_noise_min_ = static_cast<float>(get_parameter("motion_noise_min").as_double());
  motion_noise_max_ = static_cast<float>(get_parameter("motion_noise_max").as_double());

  likelihood_sigma_ = static_cast<float>(get_parameter("likelihood_sigma").as_double());
  likelihood_max_distance_ = static_cast<float>(get_parameter("likelihood_max_distance").as_double());
  likelihood_rebuild_period_ =
    std::max(1, static_cast<int>(get_parameter("likelihood_rebuild_period").as_int()));
  likelihood_min_dirty_occupied_cells_ =
    std::max(1, static_cast<int>(get_parameter("likelihood_min_dirty_occupied_cells").as_int()));

  bootstrap_min_scans_ =
    std::max(1, static_cast<int>(get_parameter("bootstrap_min_scans").as_int()));
  bootstrap_map_period_ =
    std::max(1, static_cast<int>(get_parameter("bootstrap_map_period").as_int()));
  bootstrap_min_mapped_scans_ =
    std::max(1, static_cast<int>(get_parameter("bootstrap_min_mapped_scans").as_int()));
  bootstrap_validation_limit_ =
    std::max(0, static_cast<int>(get_parameter("bootstrap_validation_scans").as_int()));

  bad_conf_limit_ = std::max(1, static_cast<int>(get_parameter("bad_conf_limit").as_int()));
  min_good_score_ = static_cast<float>(get_parameter("min_good_score").as_double());

  neff_lost_ratio_ =
    static_cast<float>(get_parameter("neff_lost_ratio").as_double());
  neff_recovered_ratio_ =
    static_cast<float>(get_parameter("neff_recovered_ratio").as_double());
  map_update_neff_ratio_ =
    static_cast<float>(get_parameter("map_update_neff_ratio").as_double());
  resample_neff_ratio_ =
    static_cast<float>(get_parameter("resample_neff_ratio").as_double());

  loop_closure_enabled_ = get_parameter("loop_closure_enabled").as_bool();
  loop_min_mapped_scans_ =
    std::max(1, static_cast<int>(get_parameter("loop_min_mapped_scans").as_int()));
  loop_min_keyframe_gap_ =
    std::max(1, static_cast<int>(get_parameter("loop_min_keyframe_gap").as_int()));
  loop_cooldown_cycles_ =
    std::max(0, static_cast<int>(get_parameter("loop_cooldown_cycles").as_int()));
  max_keyframes_ =
    std::max(1, static_cast<int>(get_parameter("max_keyframes").as_int()));
  keyframe_min_translation_ =
    static_cast<float>(get_parameter("keyframe_min_translation").as_double());
  keyframe_min_rotation_ =
    static_cast<float>(get_parameter("keyframe_min_rotation").as_double());
  loop_search_radius_ = static_cast<float>(get_parameter("loop_search_radius").as_double());
  loop_max_angle_diff_ = static_cast<float>(get_parameter("loop_max_angle_diff").as_double());
  loop_min_scan_score_ = static_cast<float>(get_parameter("loop_min_scan_score").as_double());

  min_known_free_cells_for_reset_ =
    std::max(1, static_cast<int>(get_parameter("min_known_free_cells_for_reset").as_int()));

  publish_particles_debug_ = get_parameter("publish_particles_debug").as_bool();
  particles_publish_period_ =
    std::max(1, static_cast<int>(get_parameter("particles_publish_period").as_int()));
  particles_publish_stride_ =
    std::max(1, static_cast<int>(get_parameter("particles_publish_stride").as_int()));
  map_publish_min_period_ms_ =
    std::max(0, static_cast<int>(get_parameter("map_publish_min_period_ms").as_int()));

  tracking_risk_log_delta_ =
    static_cast<float>(get_parameter("tracking_risk_log_delta").as_double());
  tracking_risk_warning_percent_ =
    static_cast<float>(get_parameter("tracking_risk_warning_percent").as_double());
  graph_visualization_enabled_ = get_parameter("graph_visualization_enabled").as_bool();

  // Safety clamps
  full_map_upload_dirty_ratio_ = std::clamp(full_map_upload_dirty_ratio_, 0.0f, 1.0f);
  neff_lost_ratio_ = std::clamp(neff_lost_ratio_, 0.0f, 1.0f);
  neff_recovered_ratio_ = std::clamp(neff_recovered_ratio_, 0.0f, 1.0f);
  map_update_neff_ratio_ = std::clamp(map_update_neff_ratio_, 0.0f, 1.0f);
  resample_neff_ratio_ = std::clamp(resample_neff_ratio_, 0.0f, 1.0f);
}

std::string SLAMNode::find_workspace_maps_directory() const
{
  namespace fs = std::filesystem;
  fs::path current = fs::current_path();

  while (!current.empty()) {
    const fs::path candidate = current / "src" / "puzzlebot_navigation" / "maps";
    if (fs::exists(candidate) && fs::is_directory(candidate)) {
      return candidate.string();
    }
    if (current == current.root_path()) {
      break;
    }
    current = current.parent_path();
  }
  return "maps";
}

void SLAMNode::synced_callback(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
  scan_msg_ = scan_msg;
  odom_msg_ = odom_msg;
  new_synced_pair_ = true;

  const Pose2D local_pose = MotionModel::pose_from_odom(*odom_msg);
  curr_odom_pose_local_ = local_pose;

  if (!odom_origin_captured_) {
    odom_origin_captured_ = true;
    prev_odom_pose_local_ = local_pose;
    motion_model_.reset_dead_reckoning(Pose2D{initial_x_, initial_y_, initial_theta_});

    RCLCPP_INFO(
      get_logger(),
      "SLAM origin captured from synchronized odometry: x=%.4f, y=%.4f, theta=%.4f | initial map pose: x=%.4f, y=%.4f, theta=%.4f",
      local_pose.x, local_pose.y, local_pose.theta,
      initial_x_, initial_y_, initial_theta_);
  }
}

void SLAMNode::initialize_particles_hybrid_known_map(float near_ratio)
{
  auto & px = particle_filter_.px();
  auto & py = particle_filter_.py();
  auto & ptheta = particle_filter_.ptheta();
  auto & weights = particle_filter_.weights();

  const auto & free_cells = map_.known_free_cells();
  if (static_cast<int>(free_cells.size()) < min_known_free_cells_for_reset_) {
    particle_filter_.initialize_particles_near(
      initial_x_, initial_y_, initial_theta_, initial_std_xy_, initial_std_theta_, rng_);
    particle_filter_.upload_particles();
    return;
  }

  std::uniform_int_distribution<int> cell_dist(0, static_cast<int>(free_cells.size()) - 1);
  std::uniform_real_distribution<float> uniform_cell(-res_ * 0.5f, res_ * 0.5f);
  std::uniform_real_distribution<float> uniform_angle(-kPi, kPi);
  std::normal_distribution<float> near_xy(0.0f, 0.35f);
  std::normal_distribution<float> near_theta(0.0f, 0.45f);

  const int n_near = corrected_pose_.has_value()
    ? static_cast<int>(static_cast<float>(N_) * near_ratio)
    : 0;
  const Pose2D cp = corrected_pose_.value_or(Pose2D{initial_x_, initial_y_, initial_theta_});

  int filled = 0;
  int trials = 0;
  const int max_trials = 30 * std::max(1, n_near);
  while (filled < n_near && trials < max_trials) {
    ++trials;
    const float x = cp.x + near_xy(rng_);
    const float y = cp.y + near_xy(rng_);
    const float th = wrap_angle(cp.theta + near_theta(rng_));
    const auto [row, col] = map_.world_to_grid(x, y);
    if (map_.inside_rc(row, col) && map_.occ_grid()[static_cast<size_t>(map_.idx(row, col))] == 0) {
      px[static_cast<size_t>(filled)] = x;
      py[static_cast<size_t>(filled)] = y;
      ptheta[static_cast<size_t>(filled)] = th;
      ++filled;
    }
  }

  while (filled < N_) {
    const auto & rc = free_cells[static_cast<size_t>(cell_dist(rng_))];
    const auto [x, y] = map_.grid_to_world(rc.first, rc.second);
    px[static_cast<size_t>(filled)] = x + uniform_cell(rng_);
    py[static_cast<size_t>(filled)] = y + uniform_cell(rng_);
    ptheta[static_cast<size_t>(filled)] = uniform_angle(rng_);
    ++filled;
  }

  const float w0 = 1.0f / static_cast<float>(N_);
  std::fill(weights.begin(), weights.end(), w0);
  particle_filter_.upload_particles();
}

void SLAMNode::run_slam_cycle()
{
  if (!new_synced_pair_ || !scan_msg_ || !odom_msg_ ||
      !prev_odom_pose_local_.has_value() || !curr_odom_pose_local_.has_value()) {
    return;
  }
  new_synced_pair_ = false;

  scan_processor_.process(*scan_msg_);
  if (!scan_processor_.valid()) {
    return;
  }
  ++total_valid_scan_counter_;

  const auto odom_delta_opt = MotionModel::compute_increment(
    prev_odom_pose_local_.value(), curr_odom_pose_local_.value());
  if (!odom_delta_opt.has_value()) {
    return;
  }
  const OdomDelta odom_delta = odom_delta_opt.value();
  motion_model_.update_dead_reckoning(odom_delta);
  const MotionNoise noise = motion_model_.compute_adaptive_noise(odom_delta, *odom_msg_);

  const SLAMState previous_state = state_machine_.state();

  particle_filter_.predict(odom_delta, noise);
  particle_filter_.upload_scan(scan_processor_.x(), scan_processor_.y(), scan_processor_.hits());
  particle_filter_.upload_map_if_needed(map_, full_map_upload_dirty_ratio_);
  const bool likelihood_rebuilt = likelihood_field_.upload_rebuilt_from_map_if_needed(map_);
  if (likelihood_rebuilt) {
    particle_filter_.upload_likelihood_field(likelihood_field_);
  }

  particle_filter_.score_and_normalize(
    scan_processor_.valid_points(), map_, likelihood_field_,
    map_manager_.mapped_scans(), bootstrap_min_mapped_scans_);

  estimated_pose_mcl_ = particle_filter_.estimate_pose();
  corrected_pose_ = graph_slam_.apply_graph_correction(estimated_pose_mcl_.value());

  if (state_machine_.state() == SLAMState::BOOTSTRAP) {
    if (state_machine_.should_bootstrap_map(total_valid_scan_counter_)) {
      map_manager_.integrate_scan(
        motion_model_.dead_reckoning_pose(),
        scan_processor_.x(), scan_processor_.y(), scan_processor_.hits());
    }
    state_machine_.bootstrap_validated(
      particle_filter_.neff(), particle_filter_.best_score(),
      map_manager_.mapped_scans(), total_valid_scan_counter_);
  } else {
    state_machine_.update_after_scores(
      particle_filter_.neff(), particle_filter_.best_score(),
      map_manager_.mapped_scans(), total_valid_scan_counter_);

    if (state_machine_.just_lost_tracking()) {
      initialize_particles_hybrid_known_map(0.0f);
      state_machine_.clear_lost_tracking_flag();

      publish_particles(true);
      publish_map(true);

      prev_odom_pose_local_ = curr_odom_pose_local_;
      return;
    }

    if (state_machine_.can_map_tracking(particle_filter_.neff(), particle_filter_.best_score()) && corrected_pose_) {
      map_manager_.integrate_scan(
        corrected_pose_.value(),
        scan_processor_.x(), scan_processor_.y(), scan_processor_.hits());
    }
  }

  if (corrected_pose_ && state_machine_.state() == SLAMState::TRACKING) {
    graph_slam_.maybe_add_keyframe(
      corrected_pose_.value(), map_manager_.mapped_scans(),
      scan_processor_.x(), scan_processor_.y(), scan_processor_.hits());

    const bool optimized = graph_slam_.maybe_detect_loop_and_optimize(
      corrected_pose_.value(),
      scan_processor_.x(), scan_processor_.y(), scan_processor_.hits(),
      map_, likelihood_field_, particle_filter_.neff(), particle_filter_.best_score(),
      min_good_score_, map_update_neff_ratio_, N_);

    if (optimized) {
      map_manager_.rebuild_from_keyframes(graph_slam_.optimized_map_node_views());
      likelihood_field_.force_rebuild_from_map(map_);
      particle_filter_.upload_map_if_needed(map_, 0.0f);
      particle_filter_.upload_likelihood_field(likelihood_field_);
      corrected_pose_ = graph_slam_.apply_graph_correction(estimated_pose_mcl_.value());
      RCLCPP_INFO(
        get_logger(),
        "GraphSLAM optimized | nodes=%zu | edges=%zu | loops_accepted=%d | loops_rejected=%d | rebuilt_map_scans=%d",
        graph_slam_.nodes().size(),
        graph_slam_.edges().size(),
        graph_slam_.accepted_loops(),
        graph_slam_.rejected_loops_count(),
        map_manager_.mapped_scans());
    }
  }

  if (state_machine_.should_resample(particle_filter_.neff())) {
    particle_filter_.resample();
  }

  if (corrected_pose_ && curr_odom_pose_local_) {
    publish_map_to_odom_tf(corrected_pose_.value(), curr_odom_pose_local_.value());
  }

  publish_map();
  publish_particles();
  publish_graph_markers();

  log_state_change_if_needed(previous_state, state_machine_.state());
  log_large_tracking_risk_change_if_needed();

  prev_odom_pose_local_ = curr_odom_pose_local_;
}

void SLAMNode::log_state_change_if_needed(SLAMState previous_state, SLAMState new_state)
{
  if (previous_state == new_state) {
    return;
  }

  const float risk = state_machine_.tracking_risk_percent(
    particle_filter_.neff(), particle_filter_.best_score());

  RCLCPP_INFO(
    get_logger(),
    "SLAM state changed | %s -> %s | tracking_risk=%.1f%% | bad_conf=%d/%d | Neff=%.1f/%.1f | score=%.3f/%.3f | valid_scans=%d | mapped_scans=%d",
    slam_state_name(previous_state), slam_state_name(new_state), risk,
    state_machine_.bad_conf_counter(), state_machine_.bad_conf_limit(),
    particle_filter_.neff(), state_machine_.neff_lost_threshold(),
    particle_filter_.best_score(), state_machine_.min_good_score(),
    total_valid_scan_counter_, map_manager_.mapped_scans());
}

void SLAMNode::log_large_tracking_risk_change_if_needed()
{
  if (state_machine_.state() != SLAMState::TRACKING) {
    return;
  }

  const float risk = state_machine_.tracking_risk_percent(
    particle_filter_.neff(), particle_filter_.best_score());

  if (last_logged_tracking_risk_ < 0.0f) {
    last_logged_tracking_risk_ = risk;
    return;
  }

  const float diff = std::abs(risk - last_logged_tracking_risk_);
  const bool large_change = diff >= tracking_risk_log_delta_;
  const bool crossed_warning =
    last_logged_tracking_risk_ < tracking_risk_warning_percent_ &&
    risk >= tracking_risk_warning_percent_;

  if (!large_change && !crossed_warning) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "TRACKING confidence changed | tracking_risk=%.1f%% | prev=%.1f%% | Neff=%.1f/%.1f | score=%.3f/%.3f",
    risk, last_logged_tracking_risk_,
    particle_filter_.neff(), state_machine_.neff_lost_threshold(),
    particle_filter_.best_score(), state_machine_.min_good_score());

  last_logged_tracking_risk_ = risk;
}

bool SLAMNode::should_publish_map_now() const
{
  const auto elapsed = now() - last_map_publish_time_;
  return elapsed.nanoseconds() >= static_cast<int64_t>(map_publish_min_period_ms_) * 1000000LL;
}

void SLAMNode::publish_map(bool force)
{
  if (!force && (!map_.map_changed() || !should_publish_map_now())) {
    return;
  }

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = now();
  msg.header.frame_id = map_frame_;
  msg.info.resolution = static_cast<double>(map_.resolution());
  msg.info.width = static_cast<uint32_t>(map_.width());
  msg.info.height = static_cast<uint32_t>(map_.height());
  msg.info.origin.position.x = map_.x_min();
  msg.info.origin.position.y = map_.y_min();
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation = yaw_to_quaternion(0.0f);
  msg.data = map_.occ_grid();

  map_pub_->publish(msg);
  map_.set_map_changed(false);
  last_map_publish_time_ = now();
}

void SLAMNode::publish_particles(bool force)
{
  ++particles_publish_counter_;
  if (!force) {
    if (!publish_particles_debug_) {
      return;
    }
    if (particles_publish_counter_ < particles_publish_period_) {
      return;
    }
  }
  particles_publish_counter_ = 0;

  particle_filter_.download_particles_strided(
    pub_px_buffer_, pub_py_buffer_, pub_ptheta_buffer_, particles_publish_stride_);

  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = now();
  msg.header.frame_id = map_frame_;
  msg.poses.reserve(pub_px_buffer_.size());

  for (size_t i = 0; i < pub_px_buffer_.size(); ++i) {
    geometry_msgs::msg::Pose p;
    p.position.x = pub_px_buffer_[i];
    p.position.y = pub_py_buffer_[i];
    p.position.z = 0.0;
    p.orientation = yaw_to_quaternion(pub_ptheta_buffer_[i]);
    msg.poses.push_back(p);
  }

  particles_pub_->publish(msg);
}

void SLAMNode::publish_map_to_odom_tf(const Pose2D & map_base, const Pose2D & odom_base)
{
  if (!tf_broadcaster_) {
    return;
  }

  const float c_odom = std::cos(odom_base.theta);
  const float s_odom = std::sin(odom_base.theta);

  const float inv_odom_x = -(c_odom * odom_base.x + s_odom * odom_base.y);
  const float inv_odom_y =  (s_odom * odom_base.x - c_odom * odom_base.y);

  const float c_map = std::cos(map_base.theta);
  const float s_map = std::sin(map_base.theta);

  const float tx = map_base.x + c_map * inv_odom_x - s_map * inv_odom_y;
  const float ty = map_base.y + s_map * inv_odom_x + c_map * inv_odom_y;
  const float yaw = wrap_angle(map_base.theta - odom_base.theta);

  geometry_msgs::msg::TransformStamped tf_msg;
  if (scan_msg_) {
    tf_msg.header.stamp = scan_msg_->header.stamp;
  } else {
    tf_msg.header.stamp = get_clock()->now();
  }
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "odom";
  tf_msg.transform.translation.x = tx;
  tf_msg.transform.translation.y = ty;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = yaw_to_quaternion(yaw);

  tf_broadcaster_->sendTransform(tf_msg);
}


void SLAMNode::publish_graph_markers()
{
  if (!graph_visualization_enabled_ || !graph_pub_) {
    return;
  }

  auto markers = graph_slam_.make_marker_array(map_frame_);
  const auto stamp = get_clock()->now();
  for (auto & marker : markers.markers) {
    marker.header.stamp = stamp;
  }
  graph_pub_->publish(markers);
}

void SLAMNode::save_map_to_pgm_yaml(const std::string & prefix)
{
  namespace fs = std::filesystem;
  fs::create_directories(map_save_dir_);

  const fs::path pgm_path = fs::path(map_save_dir_) / (prefix + ".pgm");
  const fs::path yaml_path = fs::path(map_save_dir_) / (prefix + ".yaml");

  std::vector<uint8_t> pgm(static_cast<size_t>(map_.width() * map_.height()), 205);
  const auto & occ = map_.occ_grid();
  for (size_t i = 0; i < occ.size(); ++i) {
    if (occ[i] == 0) {
      pgm[i] = 254;
    } else if (occ[i] == 100) {
      pgm[i] = 0;
    } else {
      pgm[i] = 205;
    }
  }

  {
    std::ofstream f(pgm_path, std::ios::binary);
    if (!f) {
      throw std::runtime_error("Could not open PGM file for writing");
    }
    f << "P5\n" << map_.width() << " " << map_.height() << "\n255\n";
    f.write(reinterpret_cast<const char *>(pgm.data()), static_cast<std::streamsize>(pgm.size()));
  }

  {
    std::ofstream f(yaml_path);
    if (!f) {
      throw std::runtime_error("Could not open YAML file for writing");
    }
    f
      << "image: " << pgm_path.filename().string() << "\n"
      << "mode: trinary\n"
      << "resolution: " << map_.resolution() << "\n"
      << "origin: [" << map_.x_min() << ", " << map_.y_min() << ", 0.0]\n"
      << "negate: 0\n"
      << "occupied_thresh: 0.85\n"
      << "free_thresh: 0.25\n";
  }

  RCLCPP_INFO(get_logger(), "Map saved to: %s and %s", pgm_path.string().c_str(), yaml_path.string().c_str());
}

}  // namespace puzzlebot_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<puzzlebot_navigation::SLAMNode>());
  rclcpp::shutdown();
  return 0;
}