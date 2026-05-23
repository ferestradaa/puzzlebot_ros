#pragma once

#include "puzzlebot_navigation/slam_types.hpp"

namespace puzzlebot_navigation
{

class SLAMStateMachine
{
public:
  void configure(
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
    int num_particles);

  SLAMState state() const { return state_; }
  void set_state(SLAMState s);

  bool confidence_bad(float neff, float best_score, int mapped_scans, int valid_scans) const;
  float tracking_risk_percent(float neff, float best_score) const;

  bool should_resample(float neff) const;
  bool can_map_tracking(float neff, float best_score) const;
  bool should_bootstrap_map(int valid_scan_counter) const;
  bool bootstrap_validated(float neff, float best_score, int mapped_scans, int valid_scans);

  void update_after_scores(float neff, float best_score, int mapped_scans, int valid_scans);

  bool just_lost_tracking() const { return just_lost_tracking_; }
  void clear_lost_tracking_flag() { just_lost_tracking_ = false; }

  int bad_conf_counter() const { return bad_conf_counter_; }
  int bad_conf_limit() const { return bad_conf_limit_; }
  float neff_lost_threshold() const { return neff_lost_ratio_ * static_cast<float>(N_); }
  float neff_recovered_threshold() const { return neff_recovered_ratio_ * static_cast<float>(N_); }
  float min_good_score() const { return min_good_score_; }
  int bootstrap_map_period() const { return bootstrap_map_period_; }

private:
  void reset_bootstrap_validation();

  SLAMState state_{SLAMState::BOOTSTRAP};

  bool just_lost_tracking_{false};

  int bootstrap_min_scans_{6};
  int bootstrap_min_mapped_scans_{3};
  int bootstrap_validation_limit_{4};
  int bootstrap_validation_counter_{0};
  int bootstrap_map_period_{2};

  int bad_conf_limit_{8};
  int bad_conf_counter_{0};

  float min_good_score_{0.75f};
  float neff_lost_ratio_{0.15f};
  float neff_recovered_ratio_{0.45f};
  float map_update_neff_ratio_{0.50f};
  float resample_neff_ratio_{0.65f};

  int N_{1000};
};

}  // namespace puzzlebot_navigation