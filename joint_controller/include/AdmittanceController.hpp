#pragma once

#include "MotorTypes.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

class AdmittanceController
{
public:
  explicit AdmittanceController(std::size_t n)
    : n_(n),
      tau_ext_f_(n, 0.0),
      position_ref_deg_(n, 0.0),
      mode_(n, Mode::LATCH),
      release_timer_(n, 0.0)
  {}

  // compatibility
  void setParam(double M, double B, double K) { (void)M; B_return_ = B; K_return_ = K; }
  void setDt(double dt) { dt_ = dt; }

  void setThreshold(double tau_threshold, double release_threshold)
  { tau_threshold_ = tau_threshold; release_threshold_ = release_threshold; }

  void setPositionRefDeg(const std::vector<double>& ref)
  { if (ref.size() == n_) position_ref_deg_ = ref; }

  void setCurrentLimit(double limit_mA) { current_limit_mA_ = limit_mA; }
  void setKt(double Kt) { Kt_ = Kt; }
  void setAlpha(double alpha) { alpha_ = clamp(alpha, 0.0, 0.999); }

  // keep API
  void setExternalComp(double c_ext) { c_ext_dummy_ = clamp(c_ext, 0.0, 1.0); }

  // ===== behavior tuning =====
  void setPushMode(bool enable_zero_current_push) { push_zero_current_ = enable_zero_current_push; }

  // Return: strong PD to go back fast
  void setReturnGains(double K_return, double B_return)
  {
    K_return_ = std::max(0.0, K_return);
    B_return_ = std::max(0.0, B_return);
  }

  // Mode switching hysteresis
  void setReleaseHoldTime(double seconds) { release_hold_time_ = std::max(0.0, seconds); }

  // Latch: stop controlling near ref to avoid hunting
  void setLatch(double latch_pos_deg, double latch_vel_rad_s)
  {
    latch_pos_deg_ = std::max(0.0, latch_pos_deg);
    latch_vel_rad_s_ = std::max(0.0, latch_vel_rad_s);
  }

  // If latched, re-enable RETURN
  void setUnlatch(double unlatch_pos_deg)
  {
    unlatch_pos_deg_ = std::max(0.0, unlatch_pos_deg);
  }

  void update(const std::vector<MotorState>& states, std::vector<MotorCommand>& commands)
  {
    if (states.size() != n_ || commands.size() != n_) return;

    for (std::size_t i = 0; i < n_; ++i)
    {
      const double ref_deg = position_ref_deg_[i];
      const double pos_deg = states[i].position_deg;
      const double vel_rad_s = states[i].velocity;

      const double e_deg = pos_deg - ref_deg;
      const double e_rad = deg2rad(e_deg);

      // ----- external torque estimate -----
      const double tau_ext = -states[i].current_mA * mA_to_A_ * Kt_;
      tau_ext_f_[i] = alpha_ * tau_ext_f_[i] + (1.0 - alpha_) * tau_ext;
      const double tau_abs = std::abs(tau_ext_f_[i]);

      const bool near_latch = (std::abs(e_deg) <= latch_pos_deg_) && (std::abs(vel_rad_s) <= latch_vel_rad_s_);
      const bool far_unlatch = (std::abs(e_deg) >= unlatch_pos_deg_);

      // ----- Mode transitions -----
      switch (mode_[i])
      {
        case Mode::PUSH:
        {
          // stay in PUSH while force is present
          if (tau_abs >= tau_threshold_)
          {
            release_timer_[i] = 0.0;
          }
          else
          {
            release_timer_[i] += dt_;
            if (release_timer_[i] >= release_hold_time_)
              mode_[i] = Mode::RETURN;
          }
        } break;

        case Mode::RETURN:
        {
          // if force appears again -> PUSH
          if (tau_abs >= tau_threshold_)
          {
            mode_[i] = Mode::PUSH;
            release_timer_[i] = 0.0;
          }
          else if (near_latch)
          {
            // reached reference region -> LATCH
            mode_[i] = Mode::LATCH;
          }
        } break;

        case Mode::LATCH:
        {
          // if drifts far -> leave latch
          if (tau_abs >= tau_threshold_)
          {
            mode_[i] = Mode::PUSH;
            release_timer_[i] = 0.0;
          }
          else if (far_unlatch)
          {
            mode_[i] = Mode::RETURN;
          }
        } break;
      }

      // ----- Output by mode -----
      if (mode_[i] == Mode::PUSH)
      {
        // Most compliant: command 0 current
        if (push_zero_current_)
        {
          commands[i].goal_current_mA  = 0.0;
          commands[i].current_limit_mA = current_limit_mA_;
          continue;
        }
      }

      if (mode_[i] == Mode::LATCH)
      {
        // key: no hunting
        commands[i].goal_current_mA  = 0.0;
        commands[i].current_limit_mA = current_limit_mA_;
        continue;
      }

      // Mode::RETURN (fast, non-oscillatory if B is large enough)
      const double tau_cmd = (-(K_return_ * e_rad) - (B_return_ * vel_rad_s));
      double I_cmd_mA = (tau_cmd / Kt_) * 1000.0;

      I_cmd_mA = clamp(I_cmd_mA, -current_limit_mA_, current_limit_mA_);

      commands[i].goal_current_mA  = I_cmd_mA;
      commands[i].current_limit_mA = current_limit_mA_;
    }
  }

private:
  enum class Mode { PUSH, RETURN, LATCH };

  std::size_t n_;

  // control params
  double K_return_ = 0.45;  
  double B_return_ = 0.20; 
  double dt_ = 0.02;

  // estimation
  double alpha_ = 0.85;
  double Kt_ = 0.354;
  double mA_to_A_ = 0.001;

  // push detect thresholds (Nm)
  double tau_threshold_ = 0.030;
  double release_threshold_ = 0.010;

  // mode switching
  double release_hold_time_ = 0.15;
  bool   push_zero_current_ = true;

  // latch settings (prevents hunting at ref)
  double latch_pos_deg_ = 1.0;
  double latch_vel_rad_s_ = 0.25;
  double unlatch_pos_deg_ = 3.0;

  // safety
  double current_limit_mA_ = 180.0;

  // compatibility placeholder
  double c_ext_dummy_ = 0.0;

  std::vector<double> tau_ext_f_;
  std::vector<double> position_ref_deg_;
  std::vector<Mode>   mode_;
  std::vector<double> release_timer_;

  static double deg2rad(double deg) { return deg * M_PI / 180.0; }

  static double clamp(double x, double lo, double hi)
  {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
  }
};
