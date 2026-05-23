#include "puzzlebot_navigation/graph_slam.hpp"
#include "puzzlebot_navigation/cuda_kernels.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>

#include <geometry_msgs/msg/point.hpp>

namespace puzzlebot_navigation
{

Pose2D GraphSLAM::compose_pose(const Pose2D & a, const Pose2D & b)
{
  const float c = std::cos(a.theta);
  const float s = std::sin(a.theta);

  return Pose2D{
    a.x + c * b.x - s * b.y,
    a.y + s * b.x + c * b.y,
    wrap_angle(a.theta + b.theta)};
}

Pose2D GraphSLAM::inverse_pose(const Pose2D & p)
{
  const float c = std::cos(p.theta);
  const float s = std::sin(p.theta);

  return Pose2D{
    -(c * p.x + s * p.y),
     (s * p.x - c * p.y),
    wrap_angle(-p.theta)};
}

Pose2D GraphSLAM::relative_pose(const Pose2D & from, const Pose2D & to)
{
  return compose_pose(inverse_pose(from), to);
}

Pose2D GraphSLAM::transform_between_same_pose_estimates(const Pose2D & raw, const Pose2D & opt)
{
  return compose_pose(opt, inverse_pose(raw));
}

GraphSLAM::Residual3 GraphSLAM::edge_residual(
  const Pose2D & a,
  const Pose2D & b,
  const Pose2D & z)
{
  const Pose2D predicted = relative_pose(a, b);
  return Residual3{
    predicted.x - z.x,
    predicted.y - z.y,
    wrap_angle(predicted.theta - z.theta)};
}

float GraphSLAM::residual_norm(const Residual3 & r)
{
  return std::sqrt(r.x * r.x + r.y * r.y + r.theta * r.theta);
}

float GraphSLAM::huber_weight(float error_norm, float delta)
{
  if (delta <= 1e-6f || error_norm <= delta) {
    return 1.0f;
  }
  return delta / std::max(error_norm, 1e-6f);
}

void GraphSLAM::configure(
  bool enabled,
  int min_mapped_scans,
  int min_keyframe_gap,
  int cooldown_cycles,
  int max_keyframes,
  float keyframe_min_translation,
  float keyframe_min_rotation,
  float search_radius,
  float max_angle_diff,
  float min_scan_score)
{
  enabled_ = enabled;
  min_mapped_scans_ = min_mapped_scans;
  min_keyframe_gap_ = min_keyframe_gap;
  cooldown_cycles_ = cooldown_cycles;
  max_keyframes_ = max_keyframes;
  keyframe_min_translation_ = keyframe_min_translation;
  keyframe_min_rotation_ = keyframe_min_rotation;
  search_radius_ = search_radius;
  max_angle_diff_ = max_angle_diff;
  min_scan_score_ = min_scan_score;
}

Pose2D GraphSLAM::apply_graph_correction(const Pose2D & pose) const
{
  return compose_pose(graph_correction_, pose);
}

float GraphSLAM::pose_distance_xy(const Pose2D & a, const Pose2D & b) const
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

bool GraphSLAM::should_create_keyframe(const Pose2D & pose) const
{
  if (!last_keyframe_pose_) {
    return true;
  }

  const float dxy = pose_distance_xy(pose, *last_keyframe_pose_);
  const float dth = std::abs(wrap_angle(pose.theta - last_keyframe_pose_->theta));
  return dxy >= keyframe_min_translation_ || dth >= keyframe_min_rotation_;
}

bool GraphSLAM::maybe_add_keyframe(
  const Pose2D & pose,
  int mapped_scans,
  const std::vector<float> & scan_x,
  const std::vector<float> & scan_y,
  const std::vector<uint8_t> & scan_hit)
{
  if (!enabled_ || mapped_scans < min_mapped_scans_ || !should_create_keyframe(pose)) {
    return false;
  }

  Node node;
  node.id = next_node_id_++;
  node.raw_pose = pose;
  node.optimized_pose = pose;
  node.scan_x = scan_x;
  node.scan_y = scan_y;
  node.scan_hit = scan_hit;
  int hit_count = 0;
  for (const auto h : scan_hit) {
    if (h != 0u) {
      ++hit_count;
    }
  }

  node.loop_candidate =
    hit_count >= 70 &&
    (node.id % 6 == 0);

  if (!nodes_.empty()) {
    const Node & prev = nodes_.back();

    Edge edge;
    edge.from_id = prev.id;
    edge.to_id = node.id;
    edge.measurement = relative_pose(prev.raw_pose, node.raw_pose);
    edge.information_x = 1.0f;
    edge.information_y = 1.0f;
    edge.information_theta = 1.0f;
    edge.loop_edge = false;
    edge.switchable = false;
    edge.switch_weight = 1.0f;
    edges_.push_back(edge);
  }

  nodes_.push_back(std::move(node));
  node_index_by_id_[nodes_.back().id] = nodes_.size() - 1u;
  last_keyframe_pose_ = pose;

  prune_oldest_if_needed();
  update_graph_correction_from_latest_node();
  return true;
}

void GraphSLAM::rebuild_node_index_map()
{
  node_index_by_id_.clear();
  node_index_by_id_.reserve(nodes_.size() * 2u + 1u);
  for (size_t i = 0; i < nodes_.size(); ++i) {
    node_index_by_id_[nodes_[i].id] = i;
  }
}

void GraphSLAM::prune_oldest_if_needed()
{
  if (max_keyframes_ <= 0 || static_cast<int>(nodes_.size()) <= max_keyframes_) {
    return;
  }

  const size_t remove_count = nodes_.size() - static_cast<size_t>(max_keyframes_);
  std::unordered_map<int, bool> removed_ids;
  removed_ids.reserve(remove_count * 2u + 1u);

  for (size_t i = 0; i < remove_count; ++i) {
    removed_ids[nodes_[i].id] = true;
  }

  nodes_.erase(nodes_.begin(), nodes_.begin() + static_cast<std::ptrdiff_t>(remove_count));

  edges_.erase(
    std::remove_if(edges_.begin(), edges_.end(),
      [&removed_ids](const Edge & e) {
        return removed_ids.find(e.from_id) != removed_ids.end() ||
               removed_ids.find(e.to_id) != removed_ids.end();
      }),
    edges_.end());

  rebuild_node_index_map();
}

float GraphSLAM::score_scan_at_pose_cpu(
  const Pose2D & pose,
  const std::vector<float> & scan_x,
  const std::vector<float> & scan_y,
  const std::vector<uint8_t> & scan_hit,
  const OccupancyGridMap & map,
  const LikelihoodField & likelihood,
  int & used_points) const
{
  used_points = 0;

  if (scan_x.empty() || likelihood.field().empty()) {
    return 0.0f;
  }

  const float c = std::cos(pose.theta);
  const float s = std::sin(pose.theta);
  const float sigma = std::max(0.01f, likelihood.sigma());
  const float inv_two_sigma2 = 1.0f / (2.0f * sigma * sigma);

  float sum_score = 0.0f;

  for (size_t i = 0; i < scan_x.size(); ++i) {
    if (i < scan_hit.size() && scan_hit[i] == 0u) {
      continue;
    }

    const float wx = pose.x + c * scan_x[i] - s * scan_y[i];
    const float wy = pose.y + s * scan_x[i] + c * scan_y[i];

    const auto [row, col] = map.world_to_grid(wx, wy);
    if (!map.inside_rc(row, col)) {
      continue;
    }

    const int k = map.idx(row, col);
    const float d = std::min(
      likelihood.field()[static_cast<size_t>(k)],
      likelihood.max_distance());

    sum_score += std::exp(-(d * d) * inv_two_sigma2);
    ++used_points;
  }

  if (used_points < 20) {
    return 0.0f;
  }

  return sum_score / static_cast<float>(used_points);
}

Pose2D GraphSLAM::refine_loop_measurement_scan_matching(
  const Node & candidate,
  const Node & current_node,
  const Pose2D & initial_measurement,
  const OccupancyGridMap & map,
  const LikelihoodField & likelihood,
  float & refined_score,
  int & refined_used_points) const
{
  Pose2D best_measurement = initial_measurement;
  Pose2D best_pose = compose_pose(candidate.optimized_pose, initial_measurement);

  refined_used_points = 0;
  refined_score = score_scan_at_pose_cpu(
    best_pose,
    current_node.scan_x,
    current_node.scan_y,
    current_node.scan_hit,
    map,
    likelihood,
    refined_used_points);

  const float xy_window_1 = 0.24f;
  const float th_window_1 = 0.28f;
  const float xy_step_1 = 0.08f;
  const float th_step_1 = 0.07f;

  const float xy_window_2 = 0.08f;
  const float th_window_2 = 0.10f;
  const float xy_step_2 = 0.025f;
  const float th_step_2 = 0.025f;

  auto search_window = [&](float xy_window, float th_window, float xy_step, float th_step) {
    const Pose2D center = best_measurement;

    for (float dx = -xy_window; dx <= xy_window + 1e-6f; dx += xy_step) {
      for (float dy = -xy_window; dy <= xy_window + 1e-6f; dy += xy_step) {
        for (float dth = -th_window; dth <= th_window + 1e-6f; dth += th_step) {
          Pose2D test_measurement{
            center.x + dx,
            center.y + dy,
            wrap_angle(center.theta + dth)};

          const Pose2D test_pose =
            compose_pose(candidate.optimized_pose, test_measurement);

          int used = 0;
          const float score = score_scan_at_pose_cpu(
            test_pose,
            current_node.scan_x,
            current_node.scan_y,
            current_node.scan_hit,
            map,
            likelihood,
            used);

          if (used >= 20 && score > refined_score) {
            refined_score = score;
            refined_used_points = used;
            best_measurement = test_measurement;
          }
        }
      }
    }
  };

  search_window(xy_window_1, th_window_1, xy_step_1, th_step_1);
  search_window(xy_window_2, th_window_2, xy_step_2, th_step_2);

  return best_measurement;
}

const GraphSLAM::Node * GraphSLAM::find_loop_candidate(
  const Pose2D & current_pose,
  const std::vector<float> & scan_x,
  const std::vector<float> & scan_y,
  const std::vector<uint8_t> & scan_hit,
  const OccupancyGridMap & map,
  const LikelihoodField & likelihood,
  float & best_score,
  LoopStatus & reject_reason) const
{
  best_score = 0.0f;
  reject_reason = LoopStatus::REJECTED_SCORE;

  std::vector<const Node *> candidates;
  std::vector<slam_cuda::Pose2D> candidate_poses;
  candidates.reserve(nodes_.size());
  candidate_poses.reserve(nodes_.size());

  for (const auto & node : nodes_) {
    if (!node.loop_candidate) {
      continue;
    }
    if ((next_node_id_ - node.id) < min_keyframe_gap_) {
      reject_reason = LoopStatus::REJECTED_RECENT;
      continue;
    }

    const float dxy = pose_distance_xy(current_pose, node.optimized_pose);

    if (dxy < min_loop_separation_) {
      reject_reason = LoopStatus::REJECTED_TOO_CLOSE;
      continue;
    }

    if (dxy > search_radius_) {
      reject_reason = LoopStatus::REJECTED_DISTANCE;
      continue;
    }

    const float dth = std::abs(wrap_angle(current_pose.theta - node.optimized_pose.theta));
    if (dth > max_angle_diff_) {
      reject_reason = LoopStatus::REJECTED_ANGLE;
      continue;
    }

    candidates.push_back(&node);
    candidate_poses.push_back(slam_cuda::Pose2D{
      node.optimized_pose.x,
      node.optimized_pose.y,
      node.optimized_pose.theta});
  }

  if (candidates.empty()) {
    return nullptr;
  }

  std::vector<float> scores;
  std::vector<int> used_points;
  const bool cuda_ok = slam_cuda::score_loop_candidates_cuda(
    candidate_poses,
    scan_x,
    scan_y,
    scan_hit,
    likelihood.field(),
    map.width(),
    map.height(),
    map.x_min(),
    map.y_min(),
    map.resolution(),
    likelihood.sigma(),
    likelihood.max_distance(),
    scores,
    used_points);

  const Node * best = nullptr;

  if (cuda_ok) {
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (i >= used_points.size() || used_points[i] < 20) {
        reject_reason = LoopStatus::REJECTED_NOT_ENOUGH_POINTS;
        continue;
      }

      if (scores[i] > best_score) {
        best_score = scores[i];
        best = candidates[i];
      }
    }
    return best;
  }

  // Fallback seguro: si CUDA falla, mantiene el comportamiento CPU original.
  for (const Node * node : candidates) {
    int used = 0;
    const float score = score_scan_at_pose_cpu(
      node->optimized_pose,
      scan_x,
      scan_y,
      scan_hit,
      map,
      likelihood,
      used);

    if (used < 20) {
      reject_reason = LoopStatus::REJECTED_NOT_ENOUGH_POINTS;
      continue;
    }

    if (score > best_score) {
      best_score = score;
      best = node;
    }
  }

  return best;
}

GraphSLAM::Node * GraphSLAM::find_node_by_id(int id)
{
  const auto it = node_index_by_id_.find(id);
  if (it == node_index_by_id_.end() || it->second >= nodes_.size()) {
    return nullptr;
  }
  return &nodes_[it->second];
}

const GraphSLAM::Node * GraphSLAM::find_node_by_id(int id) const
{
  const auto it = node_index_by_id_.find(id);
  if (it == node_index_by_id_.end() || it->second >= nodes_.size()) {
    return nullptr;
  }
  return &nodes_[it->second];
}

int GraphSLAM::find_node_index_by_id(int id) const
{
  const auto it = node_index_by_id_.find(id);
  if (it == node_index_by_id_.end() || it->second >= nodes_.size()) {
    return -1;
  }
  return static_cast<int>(it->second);
}

void GraphSLAM::add_rejected_loop(
  const Pose2D & from,
  const Pose2D & to,
  float score,
  LoopStatus reason)
{
  rejected_loops_.push_back(RejectedLoop{from, to, score, reason});
  ++rejected_loops_count_;

  constexpr int kMaxRejectedVisuals = 80;
  if (static_cast<int>(rejected_loops_.size()) > kMaxRejectedVisuals) {
    rejected_loops_.erase(rejected_loops_.begin());
  }
}

bool GraphSLAM::maybe_detect_loop_and_optimize(
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
  int num_particles)
{
  if (!enabled_ || nodes_.size() < 3) {
    return false;
  }

  if (cooldown_counter_ > 0) {
    --cooldown_counter_;
    return false;
  }

  if (neff < map_update_neff_ratio * static_cast<float>(num_particles) ||
      best_score < min_good_score) {
    if (!nodes_.empty()) {
      add_rejected_loop(
        current_pose,
        nodes_.back().optimized_pose,
        best_score,
        LoopStatus::REJECTED_LOW_CONFIDENCE);
    }
    return false;
  }

  float candidate_score = 0.0f;
  LoopStatus reason = LoopStatus::REJECTED_SCORE;

  const Node * candidate = find_loop_candidate(
    current_pose,
    scan_x,
    scan_y,
    scan_hit,
    map,
    likelihood,
    candidate_score,
    reason);

  if (!candidate) {
    return false;
  }

  if (candidate_score < min_scan_score_) {
    add_rejected_loop(
      current_pose,
      candidate->optimized_pose,
      candidate_score,
      LoopStatus::REJECTED_SCORE);
    return false;
  }

  const Node & current_node = nodes_.back();

  if (current_node.id == candidate->id) {
    return false;
  }

  if (last_loop_target_id_.has_value() &&
      std::abs(candidate->id - last_loop_target_id_.value()) < min_loop_target_id_separation_) {
    add_rejected_loop(
      current_pose,
      candidate->optimized_pose,
      candidate_score,
      LoopStatus::REJECTED_REPEATED_TARGET);
    return false;
  }

  const float closure_distance =
    pose_distance_xy(current_node.optimized_pose, candidate->optimized_pose);

  const float closure_angle =
    std::abs(wrap_angle(current_node.optimized_pose.theta - candidate->optimized_pose.theta));

  const float max_safe_closure_distance = std::max(0.35f, search_radius_);
  const float max_safe_closure_angle = std::max(0.50f, max_angle_diff_);

  if (closure_distance > max_safe_closure_distance ||
      closure_angle > max_safe_closure_angle) {
    add_rejected_loop(
      current_pose,
      candidate->optimized_pose,
      candidate_score,
      LoopStatus::REJECTED_DISTANCE);
    return false;
  }

  const Pose2D initial_loop_measurement =
    relative_pose(candidate->optimized_pose, current_node.optimized_pose);

  float refined_loop_score = 0.0f;
  int refined_loop_used = 0;

  const Pose2D refined_loop_measurement =
    refine_loop_measurement_scan_matching(
      *candidate,
      current_node,
      initial_loop_measurement,
      map,
      likelihood,
      refined_loop_score,
      refined_loop_used);

  if (refined_loop_used < 20 || refined_loop_score < min_scan_score_) {
    add_rejected_loop(
      current_pose,
      candidate->optimized_pose,
      refined_loop_score,
      LoopStatus::REJECTED_SCORE);
    return false;
  }

  Edge loop;
  loop.from_id = candidate->id;
  loop.to_id = current_node.id;
  loop.measurement = refined_loop_measurement;

  loop.information_x = 0.12f;
  loop.information_y = 0.12f;
  loop.information_theta = 0.08f;
  loop.loop_edge = true;
  loop.switchable = true;
  loop.switch_weight = 1.0f;
  edges_.push_back(loop);

  ++accepted_loops_;
  last_loop_target_id_ = candidate->id;

  last_optimized_from_id_ = candidate->id;
  last_optimized_to_id_ = current_node.id;

  cooldown_counter_ = cooldown_cycles_;

  optimize();
  update_graph_correction_from_latest_node();

  return true;
}

double GraphSLAM::graph_cost() const
{
  double cost = 0.0;

  for (const auto & edge : edges_) {
    const Node * a = find_node_by_id(edge.from_id);
    const Node * b = find_node_by_id(edge.to_id);
    if (!a || !b) {
      continue;
    }

    const Residual3 r = edge_residual(a->optimized_pose, b->optimized_pose, edge.measurement);
    const float norm = residual_norm(r);
    const float robust = huber_weight(norm, robust_kernel_delta_);
    const float sw = edge.switchable ? edge.switch_weight : 1.0f;

    const double wx = static_cast<double>(edge.information_x) * robust * sw;
    const double wy = static_cast<double>(edge.information_y) * robust * sw;
    const double wt = static_cast<double>(edge.information_theta) * robust * sw;

    cost += 0.5 * (
      wx * static_cast<double>(r.x * r.x) +
      wy * static_cast<double>(r.y * r.y) +
      wt * static_cast<double>(r.theta * r.theta));
  }

  return cost;
}

bool GraphSLAM::solve_sparse_linear_system(
  const Eigen::SparseMatrix<double> & A,
  const Eigen::VectorXd & b,
  std::vector<double> & x) const
{
  x.assign(static_cast<size_t>(b.size()), 0.0);

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> chol;
  chol.compute(A);
  if (chol.info() == Eigen::Success) {
    const Eigen::VectorXd solution = chol.solve(b);
    if (chol.info() == Eigen::Success && solution.allFinite()) {
      x.assign(solution.data(), solution.data() + solution.size());
      return true;
    }
  }

  // Fallback for near-singular systems. Slower than Cholesky but still sparse.
  Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> cg;
  cg.setTolerance(1e-9);
  cg.setMaxIterations(std::max<int>(200, static_cast<int>(b.size()) * 4));
  cg.compute(A);
  if (cg.info() != Eigen::Success) {
    return false;
  }

  const Eigen::VectorXd solution = cg.solve(b);
  if (cg.info() != Eigen::Success || !solution.allFinite()) {
    return false;
  }

  x.assign(solution.data(), solution.data() + solution.size());
  return true;
}

void GraphSLAM::update_switchable_weights()
{
  for (auto & edge : edges_) {
    if (!edge.switchable) {
      edge.switch_weight = 1.0f;
      continue;
    }

    const Node * a = find_node_by_id(edge.from_id);
    const Node * b = find_node_by_id(edge.to_id);
    if (!a || !b) {
      edge.switch_weight = min_switch_weight_;
      continue;
    }

    const Residual3 r = edge_residual(a->optimized_pose, b->optimized_pose, edge.measurement);
    const float e = residual_norm(r);
    const float scale = std::max(1e-3f, switch_error_scale_);

    edge.switch_weight = std::clamp(
      1.0f / (1.0f + (e * e) / (scale * scale)),
      min_switch_weight_,
      1.0f);
  }
}

void GraphSLAM::optimize(double tolerance)
{
  if (nodes_.size() < 2 || edges_.empty()) {
    return;
  }

  const int n_nodes = static_cast<int>(nodes_.size());
  const int anchor_index = 0;
  const int variables = 3 * (n_nodes - 1);

  if (variables <= 0) {
    return;
  }

  double lambda = lm_lambda_initial_;

  while (true) {
    update_switchable_weights();

    const double current_cost = graph_cost();

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<size_t>(edges_.size()) * 36u + static_cast<size_t>(variables));

    Eigen::VectorXd g = Eigen::VectorXd::Zero(variables);

    auto variable_index = [anchor_index](int node_index, int dim) -> int {
      if (node_index <= anchor_index) {
        return -1;
      }
      return 3 * (node_index - 1) + dim;
    };

    auto add_jacobian_block =
      [&](const Edge & edge, int node_index, int dim, const Residual3 & base_r,
          double J[3])
      {
        if (node_index <= anchor_index) {
          J[0] = 0.0;
          J[1] = 0.0;
          J[2] = 0.0;
          return;
        }

        const int node_id = nodes_[static_cast<size_t>(node_index)].id;
        const bool is_a = node_id == edge.from_id;
        const bool is_b = node_id == edge.to_id;

        if (!is_a && !is_b) {
          J[0] = 0.0;
          J[1] = 0.0;
          J[2] = 0.0;
          return;
        }

        const float eps =
          (dim == 2) ? lm_numeric_eps_theta_ : lm_numeric_eps_xy_;

        Node * perturb_node = find_node_by_id(node_id);
        if (!perturb_node) {
          J[0] = 0.0;
          J[1] = 0.0;
          J[2] = 0.0;
          return;
        }

        const Pose2D backup = perturb_node->optimized_pose;

        if (dim == 0) {
          perturb_node->optimized_pose.x += eps;
        } else if (dim == 1) {
          perturb_node->optimized_pose.y += eps;
        } else {
          perturb_node->optimized_pose.theta =
            wrap_angle(perturb_node->optimized_pose.theta + eps);
        }

        const Node * a = find_node_by_id(edge.from_id);
        const Node * b = find_node_by_id(edge.to_id);

        Residual3 perturbed{};
        if (a && b) {
          perturbed = edge_residual(a->optimized_pose, b->optimized_pose, edge.measurement);
        }

        perturb_node->optimized_pose = backup;

        J[0] = (static_cast<double>(perturbed.x) - static_cast<double>(base_r.x)) /
          static_cast<double>(eps);
        J[1] = (static_cast<double>(perturbed.y) - static_cast<double>(base_r.y)) /
          static_cast<double>(eps);
        J[2] = static_cast<double>(wrap_angle(perturbed.theta - base_r.theta)) /
          static_cast<double>(eps);
      };

    for (const auto & edge : edges_) {
      const int ia = find_node_index_by_id(edge.from_id);
      const int ib = find_node_index_by_id(edge.to_id);
      if (ia < 0 || ib < 0) {
        continue;
      }

      const Node * a = find_node_by_id(edge.from_id);
      const Node * b = find_node_by_id(edge.to_id);
      if (!a || !b) {
        continue;
      }

      const Residual3 r = edge_residual(a->optimized_pose, b->optimized_pose, edge.measurement);
      const float norm = residual_norm(r);
      const float robust = huber_weight(norm, robust_kernel_delta_);
      const float sw = edge.switchable ? edge.switch_weight : 1.0f;

      const double weights[3] = {
        static_cast<double>(edge.information_x) * robust * sw,
        static_cast<double>(edge.information_y) * robust * sw,
        static_cast<double>(edge.information_theta) * robust * sw
      };

      const double residual[3] = {
        static_cast<double>(r.x),
        static_cast<double>(r.y),
        static_cast<double>(r.theta)
      };

      const int involved_nodes[2] = {ia, ib};

      double J_blocks[2][3][3] = {};

      for (int block = 0; block < 2; ++block) {
        for (int dim = 0; dim < 3; ++dim) {
          add_jacobian_block(edge, involved_nodes[block], dim, r, J_blocks[block][dim]);
        }
      }

      for (int block_i = 0; block_i < 2; ++block_i) {
        for (int dim_i = 0; dim_i < 3; ++dim_i) {
          const int vi = variable_index(involved_nodes[block_i], dim_i);
          if (vi < 0) {
            continue;
          }

          for (int residual_dim = 0; residual_dim < 3; ++residual_dim) {
            g[vi] +=
              J_blocks[block_i][dim_i][residual_dim] *
              weights[residual_dim] *
              residual[residual_dim];
          }

          for (int block_j = 0; block_j < 2; ++block_j) {
            for (int dim_j = 0; dim_j < 3; ++dim_j) {
              const int vj = variable_index(involved_nodes[block_j], dim_j);
              if (vj < 0) {
                continue;
              }

              double value = 0.0;
              for (int residual_dim = 0; residual_dim < 3; ++residual_dim) {
                value +=
                  J_blocks[block_i][dim_i][residual_dim] *
                  weights[residual_dim] *
                  J_blocks[block_j][dim_j][residual_dim];
              }

              if (std::abs(value) > 1e-18) {
                triplets.emplace_back(vi, vj, value);
              }
            }
          }
        }
      }
    }

    for (int i = 0; i < variables; ++i) {
      triplets.emplace_back(i, i, lambda + 1e-9);
      g[i] = -g[i];
    }

    Eigen::SparseMatrix<double> H(variables, variables);
    H.setFromTriplets(triplets.begin(), triplets.end());
    H.makeCompressed();

    std::vector<double> dx;
    if (!solve_sparse_linear_system(H, g, dx)) {
      lambda *= lm_lambda_up_;
      continue;
    }

    std::vector<Pose2D> backup;
    backup.reserve(nodes_.size());
    for (const auto & node : nodes_) {
      backup.push_back(node.optimized_pose);
    }

    for (int node_index = 1; node_index < n_nodes; ++node_index) {
      const int base = 3 * (node_index - 1);

      nodes_[static_cast<size_t>(node_index)].optimized_pose.x +=
        static_cast<float>(dx[static_cast<size_t>(base + 0)]);
      nodes_[static_cast<size_t>(node_index)].optimized_pose.y +=
        static_cast<float>(dx[static_cast<size_t>(base + 1)]);
      nodes_[static_cast<size_t>(node_index)].optimized_pose.theta =
        wrap_angle(
          nodes_[static_cast<size_t>(node_index)].optimized_pose.theta +
          static_cast<float>(dx[static_cast<size_t>(base + 2)]));
    }

    update_switchable_weights();
    const double new_cost = graph_cost();
    const double error = std::abs(current_cost - new_cost);

    if (new_cost < current_cost) {
      lambda = std::max(1e-9, lambda * static_cast<double>(lm_lambda_down_));
    } else {
      for (size_t i = 0; i < nodes_.size(); ++i) {
        nodes_[i].optimized_pose = backup[i];
      }

      lambda *= static_cast<double>(lm_lambda_up_);
    }

    if (error < tolerance) {
      break;
    }
  }
}

void GraphSLAM::update_graph_correction_from_latest_node()
{
  if (nodes_.empty()) {
    graph_correction_ = Pose2D{};
    return;
  }

  const Node & last = nodes_.back();
  graph_correction_ = transform_between_same_pose_estimates(
    last.raw_pose,
    last.optimized_pose);
}

std::vector<GraphMapNodeView> GraphSLAM::optimized_map_node_views() const
{
  std::vector<GraphMapNodeView> views;
  views.reserve(nodes_.size());

  for (const auto & node : nodes_) {
    views.push_back(GraphMapNodeView{
      node.optimized_pose,
      &node.scan_x,
      &node.scan_y,
      &node.scan_hit});
  }

  return views;
}

static geometry_msgs::msg::Point point_from_pose(const Pose2D & p, double z)
{
  geometry_msgs::msg::Point out;
  out.x = p.x;
  out.y = p.y;
  out.z = z;
  return out;
}

visualization_msgs::msg::MarkerArray GraphSLAM::make_marker_array(
  const std::string & frame_id) const
{
  visualization_msgs::msg::MarkerArray out;

  auto make_base = [&](int id, const std::string & ns, int type) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    return m;
  };

  auto raw_path = make_base(
    0,
    "raw_trajectory_blue",
    visualization_msgs::msg::Marker::LINE_STRIP);
  raw_path.scale.x = 0.025;
  raw_path.color.b = 1.0f;
  raw_path.color.a = 0.95f;

  auto opt_path = make_base(
    1,
    "optimized_trajectory_green",
    visualization_msgs::msg::Marker::LINE_STRIP);
  opt_path.scale.x = 0.040;
  opt_path.color.g = 1.0f;
  opt_path.color.a = 0.95f;

  auto nodes_marker = make_base(
    2,
    "keyframe_nodes_yellow",
    visualization_msgs::msg::Marker::SPHERE_LIST);
  nodes_marker.scale.x = 0.10;
  nodes_marker.scale.y = 0.10;
  nodes_marker.scale.z = 0.10;
  nodes_marker.color.r = 1.0f;
  nodes_marker.color.g = 0.85f;
  nodes_marker.color.a = 0.95f;

  auto loop_candidate_nodes = make_base(
    6,
    "loop_candidate_nodes_purple",
    visualization_msgs::msg::Marker::SPHERE_LIST);
  loop_candidate_nodes.scale.x = 0.16;
  loop_candidate_nodes.scale.y = 0.16;
  loop_candidate_nodes.scale.z = 0.16;
  loop_candidate_nodes.color.r = 0.65f;
  loop_candidate_nodes.color.b = 1.0f;
  loop_candidate_nodes.color.a = 0.95f;

  for (const auto & node : nodes_) {
    raw_path.points.push_back(point_from_pose(node.raw_pose, 0.05));
    opt_path.points.push_back(point_from_pose(node.optimized_pose, 0.08));

    nodes_marker.points.push_back(point_from_pose(node.optimized_pose, 0.12));

    if (node.loop_candidate) {
      loop_candidate_nodes.points.push_back(point_from_pose(node.optimized_pose, 0.20));
    }
  }

  out.markers.push_back(raw_path);
  out.markers.push_back(opt_path);
  out.markers.push_back(nodes_marker);
  out.markers.push_back(loop_candidate_nodes);

  return out;
}

}  // namespace puzzlebot_navigation