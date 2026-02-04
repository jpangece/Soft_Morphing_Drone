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
      I_cmd_f_mA_(n, 0.0),
      push_mode_(n, false),
      release_timer_(n, 0.0)
  {}

  void setParam(double M, double B, double K) { M_ = M; B_return_ = B; K_return_ = K; }
  void setDt(double dt) { dt_ = dt; }

  void setThreshold(double tau_threshold, double release_threshold)
  { tau_threshold_ = tau_threshold; release_threshold_ = release_threshold; }

  void setPositionRefDeg(const std::vector<double>& ref)
  { if (ref.size() == n_) position_ref_deg_ = ref; }

  void setCurrentLimit(double limit_mA) { current_limit_mA_ = limit_mA; }
  void setKt(double Kt) { Kt_ = Kt; }
  void setAlpha(double alpha) { alpha_ = clamp(alpha, 0.0, 0.999); }

  // When pushing: soft (low K, low B)
  void setPushGains(double K_push, double B_push)
  {
    K_push_ = std::max(0.0, K_push);
    B_push_ = std::max(0.0, B_push);
  }

  // When released, use critical damping (B_return = 2*sqrt(K_return*Mvirt))
  void setReturnGains(double K_return, double B_return)
  {
    K_return_ = std::max(0.0, K_return);
    B_return_ = std::max(0.0, B_return);
  }

  // Hysteresis + hold time to avoid chattering between modes
  void setReleaseHoldTime(double seconds) { release_hold_time_ = std::max(0.0, seconds); }

  // Command smoothing
  void setCmdFilterAlpha(double a) { cmd_alpha_ = clamp(a, 0.0, 0.999); }

  // If close enough AND slow, command 0 current
  void setDeadband(double pos_deadband_deg, double vel_deadband_rad_s)
  {
    pos_deadband_deg_ = std::max(0.0, pos_deadband_deg);
    vel_deadband_rad_s_ = std::max(0.0, vel_deadband_rad_s);
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

      // ===== external torque estimate from current (Nm) =====
      const double tau_ext = -states[i].current_mA * mA_to_A_ * Kt_;
      tau_ext_f_[i] = alpha_ * tau_ext_f_[i] + (1.0 - alpha_) * tau_ext;
      const double tau_abs = std::abs(tau_ext_f_[i]);

      // ===== push/release mode decision with hysteresis + hold =====
      // If tau is large: push
      if (tau_abs >= tau_threshold_)
      {
        push_mode_[i] = true;
        release_timer_[i] = 0.0;
      }
      else
      {
        // tau is small: count time since release
        release_timer_[i] += dt_;
        if (release_timer_[i] >= release_hold_time_)
          push_mode_[i] = false;
      }

      // ===== deadband to prevent tiny oscillation near reference =====
      if (std::abs(e_deg) <= pos_deadband_deg_ && std::abs(vel_rad_s) <= vel_deadband_rad_s_)
      {
        I_cmd_f_mA_[i] = 0.0;
        commands[i].goal_current_mA  = 0.0;
        commands[i].current_limit_mA = current_limit_mA_;
        continue;
      }

      // ===== choose gains by mode =====
      const double K_use = push_mode_[i] ? K_push_ : K_return_;
      const double B_use = push_mode_[i] ? B_push_ : B_return_;

      // ===== core control: pure spring-damper to ref =====
      const double tau_cmd = (-(K_use * e_rad) - (B_use * vel_rad_s));

      // torque -> current (mA)
      double I_cmd_mA = (tau_cmd / Kt_) * 1000.0;

      // clamp safety
      I_cmd_mA = clamp(I_cmd_mA, -current_limit_mA_, current_limit_mA_);

      // smooth command
      I_cmd_f_mA_[i] = cmd_alpha_ * I_cmd_f_mA_[i] + (1.0 - cmd_alpha_) * I_cmd_mA;

      commands[i].goal_current_mA  = I_cmd_f_mA_[i];
      commands[i].current_limit_mA = current_limit_mA_;
    }
  }

private:
  std::size_t n_;

  double M_ = 0.05;

  // ===== Return gains (strong) =====
  // These are the ones you tune for "fast return to zero without oscillation".
  double K_return_ = 0.35;     // Nm/rad
  double B_return_ = 0.12;     // Nm/(rad/s)

  // ===== Push gains (soft) =====
  // These are the ones you tune for "easy to push by hand".
  double K_push_   = 0.05;     // Nm/rad
  double B_push_   = 0.02;     // Nm/(rad/s)

  double dt_ = 0.02;

  // external torque estimation
  double alpha_ = 0.85;
  double Kt_ = 0.354;
  double mA_to_A_ = 0.001;

  // thresholds (Nm) for push detection
  double tau_threshold_ = 0.035;
  double release_threshold_ = 0.010; // kept but not used directly; left for compatibility

  // time to stay in push mode after force disappears (prevents chattering)
  double release_hold_time_ = 0.20; // seconds

  // safety
  double current_limit_mA_ = 180.0;

  // deadband near ref (prevents tiny hunting)
  double pos_deadband_deg_ = 0.6;
  double vel_deadband_rad_s_ = 0.20;

  // command LPF
  double cmd_alpha_ = 0.6;

  std::vector<double> tau_ext_f_;
  std::vector<double> position_ref_deg_;
  std::vector<double> I_cmd_f_mA_;
  std::vector<bool>   push_mode_;
  std::vector<double> release_timer_;

  static double deg2rad(double deg) { return deg * M_PI / 180.0; }

  static double clamp(double x, double lo, double hi)
  {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
  }
};
