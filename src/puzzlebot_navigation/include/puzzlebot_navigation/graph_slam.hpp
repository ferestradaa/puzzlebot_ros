#pragma once

#include "puzzlebot_navigation/slam_types.hpp"
#include "puzzlebot_navigation/occupancy_grid_map.hpp"
#include "puzzlebot_navigation/likelihood_field.hpp"
#include "puzzlebot_navigation/map_manager.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Sparse>

namespace puzzlebot_navigation
{

class GraphSLAM
{
public:
  enum class LoopStatus
  {
    ACCEPTED = 0,
    REJECTED_RECENT,
    REJECTED_DISTANCE,
    REJECTED_ANGLE,
    REJECTED_SCORE,
    REJECTED_NOT_ENOUGH_POINTS,
    REJECTED_LOW_CONFIDENCE,
    REJECTED_REPEATED_TARGET,
    REJECTED_TOO_CLOSE
  };

  struct Node
  {
    int id{0};
    Pose2D raw_pose{};
    Pose2D optimized_pose{};
    std::vector<float> scan_x;
    std::vector<float> scan_y;
    std::vector<uint8_t> scan_hit;
    bool loop_candidate{false};
  };

  struct Edge
  {
    int from_id{0};
    int to_id{0};
    Pose2D measurement{};
    float information_x{1.0f};
    float information_y{1.0f};
    float information_theta{1.0f};
    bool loop_edge{false};
    bool switchable{false};
    float switch_weight{1.0f};
  };

  struct RejectedLoop
  {
    Pose2D from_pose{};
    Pose2D to_pose{};
    float score{0.0f};
    LoopStatus reason{LoopStatus::REJECTED_SCORE};
  };

  void configure(
    bool enabled,
    int min_mapped_scans,
    int min_keyframe_gap,
    int cooldown_cycles,
    int max_keyframes,
    float keyframe_min_translation,
    float keyframe_min_rotation,
    float search_radius,
    float max_angle_diff,
    float min_scan_score);

  Pose2D apply_graph_correction(const Pose2D & pose) const;

  bool maybe_add_keyframe(
    const Pose2D & pose,
    int mapped_scans,
    const std::vector<float> & scan_x,
    const std::vector<float> & scan_y,
    const std::vector<uint8_t> & scan_hit);

  bool maybe_detect_loop_and_optimize(
    const Pose2D & current_pose,
    const std::vector<float> & scan_x,
    const std::vector<float> & scan_y,
    const std::vector<uint8_t> & scan_hit,
    const OccupancyGridMap & map,
    const LikelihoodField & likelihood,
    float neff,
    float best_score,
    float min_good_score,
    float map_update_neff_ratio,
    int num_particles);

  visualization_msgs::msg::MarkerArray make_marker_array(const std::string & frame_id) const;
  std::vector<GraphMapNodeView> optimized_map_node_views() const;

  const std::vector<Node> & nodes() const { return nodes_; }
  const std::vector<Edge> & edges() const { return edges_; }
  int accepted_loops() const { return accepted_loops_; }
  int rejected_loops_count() const { return rejected_loops_count_; }

private:
  struct Residual3
  {
    float x{0.0f};
    float y{0.0f};
    float theta{0.0f};
  };

  static Pose2D compose_pose(const Pose2D & a, const Pose2D & b);
  static Pose2D inverse_pose(const Pose2D & p);
  static Pose2D relative_pose(const Pose2D & from, const Pose2D & to);
  static Pose2D transform_between_same_pose_estimates(const Pose2D & raw, const Pose2D & opt);

  static Residual3 edge_residual(const Pose2D & a, const Pose2D & b, const Pose2D & z);
  static float residual_norm(const Residual3 & r);
  static float huber_weight(float error_norm, float delta);

  float pose_distance_xy(const Pose2D & a, const Pose2D & b) const;
  bool should_create_keyframe(const Pose2D & pose) const;
  void prune_oldest_if_needed();

  float score_scan_at_pose_cpu(
    const Pose2D & pose,
    const std::vector<float> & scan_x,
    const std::vector<float> & scan_y,
    const std::vector<uint8_t> & scan_hit,
    const OccupancyGridMap & map,
    const LikelihoodField & likelihood,
    int & used_points) const;

  Pose2D refine_loop_measurement_scan_matching(
    const Node & candidate,
    const Node & current_node,
    const Pose2D & initial_measurement,
    const OccupancyGridMap & map,
    const LikelihoodField & likelihood,
    float & refined_score,
    int & refined_used_points) const;
    
  const Node * find_loop_candidate(
    const Pose2D & current_pose,
    const std::vector<float> & scan_x,
    const std::vector<float> & scan_y,
    const std::vector<uint8_t> & scan_hit,
    const OccupancyGridMap & map,
    const LikelihoodField & likelihood,
    float & best_score,
    LoopStatus & reject_reason) const;

  void rebuild_node_index_map();
  Node * find_node_by_id(int id);
  const Node * find_node_by_id(int id) const;
  int find_node_index_by_id(int id) const;

  void add_rejected_loop(const Pose2D & from, const Pose2D & to, float score, LoopStatus reason);

  double graph_cost() const;
  bool solve_sparse_linear_system(
    const Eigen::SparseMatrix<double> & A,
    const Eigen::VectorXd & b,
    std::vector<double> & x) const;

  void optimize(double tolerance = 1e-6);
  void update_switchable_weights();
  void update_graph_correction_from_latest_node();

private:
  bool enabled_{true};
  int min_mapped_scans_{12};
  int min_keyframe_gap_{10};
  int cooldown_cycles_{12};
  int cooldown_counter_{0};
  int max_keyframes_{300};

  float keyframe_min_translation_{0.35f};
  float keyframe_min_rotation_{0.35f};
  float search_radius_{0.75f};
  float max_angle_diff_{0.85f};
  float min_scan_score_{0.76f};

  float min_loop_separation_{0.1f};
  int min_loop_target_id_separation_{8};
  std::optional<int> last_loop_target_id_;
  std::optional<int> last_optimized_from_id_;
  std::optional<int> last_optimized_to_id_;

  float lm_lambda_initial_{1e-3f};
  float lm_lambda_up_{10.0f};
  float lm_lambda_down_{0.25f};
  float lm_numeric_eps_xy_{1e-3f};
  float lm_numeric_eps_theta_{1e-3f};

  float robust_kernel_delta_{0.25f};
  float switch_error_scale_{0.20f};
  float min_switch_weight_{0.02f};

  int next_node_id_{0};
  std::optional<Pose2D> last_keyframe_pose_;

  std::vector<Node> nodes_;
  std::unordered_map<int, size_t> node_index_by_id_;
  std::vector<Edge> edges_;
  std::vector<RejectedLoop> rejected_loops_;

  Pose2D graph_correction_{};

  int accepted_loops_{0};
  int rejected_loops_count_{0};
};

}  // namespace puzzlebot_navigation