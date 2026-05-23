#include "puzzlebot_navigation/slam_state_machine.hpp"

#include <algorithm>

namespace puzzlebot_navigation
{

void SLAMStateMachine::configure(
  int bootstrap_min_scans,
  int bootstrap_min_mapped_scans,
  int bootstrap_validation_scans,
  int bootstrap_map_period,
  int bad_conf_limit,
  float min_good_score,
  float neff_lost_ratio,
  float neff_recovered_ratio,
  float map_update_neff_ratio,
  float resample_neff_ratio,
  int num_particles)
{
  bootstrap_min_scans_ = bootstrap_min_scans;
  bootstrap_min_mapped_scans_ = bootstrap_min_mapped_scans;
  bootstrap_validation_limit_ = bootstrap_validation_scans;
  bootstrap_map_period_ = bootstrap_map_period;

  bad_conf_limit_ = bad_conf_limit;

  min_good_score_ = min_good_score;
  neff_lost_ratio_ = neff_lost_ratio;
  neff_recovered_ratio_ = neff_recovered_ratio;
  map_update_neff_ratio_ = map_update_neff_ratio;
  resample_neff_ratio_ = resample_neff_ratio;

  N_ = num_particles;
}

void SLAMStateMachine::set_state(SLAMState s)
{
  state_ = s;

  if (state_ == SLAMState::BOOTSTRAP) {
    reset_bootstrap_validation();
  }

  if (state_ == SLAMState::TRACKING) {
    bad_conf_counter_ = 0;
    just_lost_tracking_ = false;
  }
}

bool SLAMStateMachine::confidence_bad(
  float neff,
  float best_score,
  int mapped_scans,
  int valid_scans) const
{
  if (mapped_scans < bootstrap_min_mapped_scans_ || valid_scans < bootstrap_min_scans_) {
    return false;
  }

  const bool neff_bad = neff < neff_lost_ratio_ * static_cast<float>(N_);
  const bool score_bad = best_score < min_good_score_;

  return neff_bad || score_bad;
}

float SLAMStateMachine::tracking_risk_percent(float neff, float best_score) const
{
  float score_risk = 0.0f;
  if (min_good_score_ > 1e-6f) {
    score_risk = 100.0f * (1.0f - best_score / min_good_score_);
  }

  const float neff_threshold = neff_lost_ratio_ * static_cast<float>(N_);
  float neff_risk = 0.0f;
  if (neff_threshold > 1e-6f) {
    neff_risk = 100.0f * (1.0f - neff / neff_threshold);
  }

  return std::max(
    std::clamp(score_risk, 0.0f, 100.0f),
    std::clamp(neff_risk, 0.0f, 100.0f));
}

bool SLAMStateMachine::should_resample(float neff) const
{
  return neff < resample_neff_ratio_ * static_cast<float>(N_);
}

bool SLAMStateMachine::can_map_tracking(float neff, float best_score) const
{
  return state_ == SLAMState::TRACKING &&
    neff > map_update_neff_ratio_ * static_cast<float>(N_) &&
    best_score > min_good_score_;
}

bool SLAMStateMachine::should_bootstrap_map(int valid_scan_counter) const
{
  return state_ == SLAMState::BOOTSTRAP &&
    bootstrap_map_period_ > 0 &&
    (valid_scan_counter % bootstrap_map_period_) == 0;
}

bool SLAMStateMachine::bootstrap_validated(
  float neff,
  float best_score,
  int mapped_scans,
  int valid_scans)
{
  if (state_ != SLAMState::BOOTSTRAP) {
    return false;
  }

  if (mapped_scans < bootstrap_min_mapped_scans_ || valid_scans < bootstrap_min_scans_) {
    return false;
  }

  const bool confident =
    neff < neff_recovered_ratio_ * static_cast<float>(N_) &&
    best_score > min_good_score_;

  if (confident) {
    ++bootstrap_validation_counter_;
  } else {
    bootstrap_validation_counter_ = 0;
  }

  if (bootstrap_validation_counter_ >= bootstrap_validation_limit_) {
    state_ = SLAMState::TRACKING;
    bad_conf_counter_ = 0;
    just_lost_tracking_ = false;
    reset_bootstrap_validation();
    return true;
  }

  return false;
}

void SLAMStateMachine::update_after_scores(
  float neff,
  float best_score,
  int mapped_scans,
  int valid_scans)
{
  just_lost_tracking_ = false;

  if (state_ != SLAMState::TRACKING) {
    return;
  }

  if (confidence_bad(neff, best_score, mapped_scans, valid_scans)) {
    ++bad_conf_counter_;
  } else {
    bad_conf_counter_ = 0;
  }

  if (bad_conf_counter_ >= bad_conf_limit_) {
    state_ = SLAMState::BOOTSTRAP;
    bad_conf_counter_ = 0;
    reset_bootstrap_validation();
    just_lost_tracking_ = true;
  }
}

void SLAMStateMachine::reset_bootstrap_validation()
{
  bootstrap_validation_counter_ = 0;
}

}  // namespace puzzlebot_navigation