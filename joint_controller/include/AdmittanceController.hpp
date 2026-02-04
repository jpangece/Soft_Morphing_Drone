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
      position_ref_deg_(n, 0.0),
      e_prev_deg_(n, 0.0),
      latched_(n, true),
      latch_timer_(n, 0.0)
  {}

  // compatibility
  void setParam(double /*M*/, double B, double K) { B_return_ = B; K_return_ = K; }
  void setDt(double dt) { dt_ = dt; }

  void setPositionRefDeg(const std::vector<double>& ref)
  { if (ref.size() == n_) position_ref_deg_ = ref; }

  void setCurrentLimit(double limit_mA) { current_limit_mA_ = std::max(0.0, limit_mA); }
  void setKt(double Kt) { Kt_ = std::max(1e-6, Kt); }

  // keep API
  void setAlpha(double /*alpha*/) {}
  void setThreshold(double /*tau_threshold*/, double /*release_threshold*/) {}
  void setExternalComp(double c_ext) { c_ext_dummy_ = clamp(c_ext, 0.0, 1.0); }

  // RETURN gains
  void setReturnGains(double K_return, double B_return)
  {
    K_return_ = std::max(0.0, K_return);
    B_return_ = std::max(0.0, B_return);
  }

  // PUSH detection
  // If |vel| is large and |error| is increasing -> treat as user pushing.
  void setPushDetect(double vel_push_rad_s, double e_grow_deg)
  {
    vel_push_rad_s_ = std::max(0.0, vel_push_rad_s);
    e_grow_deg_ = std::max(0.0, e_grow_deg);
  }

  // LATCH capture near zero to kill oscillation:
  void setLatchWindow(double latch_pos_deg, double latch_vel_rad_s)
  {
    latch_pos_deg_ = std::max(0.0, latch_pos_deg);
    latch_vel_rad_s_ = std::max(0.0, latch_vel_rad_s);
  }

  // Zero-crossing capture
  void setZeroCrossCapture(double capture_pos_deg)
  {
    capture_pos_deg_ = std::max(0.0, capture_pos_deg);
  }

  // If latched but drifted away, unlock and return again
  void setUnlatch(double unlatch_pos_deg)
  {
    unlatch_pos_deg_ = std::max(0.0, unlatch_pos_deg);
  }

  void setLatchHoldTime(double seconds)
  {
    latch_hold_time_ = std::max(0.0, seconds);
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

      // ---- latch drift logic ----
      if (latched_[i])
      {
        latch_timer_[i] += dt_;

        // stay latched for a short hold time
        const bool hold_ok = (latch_timer_[i] >= latch_hold_time_);

        if (hold_ok && std::abs(e_deg) >= unlatch_pos_deg_)
        {
          // drifted away -> unlock and return
          latched_[i] = false;
          latch_timer_[i] = 0.0;
        }
        else
        {
          // latched: always command 0 current
          commands[i].goal_current_mA  = 0.0;
          commands[i].current_limit_mA = current_limit_mA_;
          e_prev_deg_[i] = e_deg;
          continue;
        }
      }

      // ---- PUSH detection ----
      const bool err_growing = (std::abs(e_deg) > std::abs(e_prev_deg_[i]) + e_grow_deg_);
      const bool fast_motion = (std::abs(vel_rad_s) >= vel_push_rad_s_);

      if (err_growing && fast_motion)
      {
        commands[i].goal_current_mA  = 0.0;
        commands[i].current_limit_mA = current_limit_mA_;
        e_prev_deg_[i] = e_deg;
        continue;
      }

      // ---- Zero-crossing capture ----
      const double e_prev = e_prev_deg_[i];
      const bool crossed_zero = (e_prev > 0.0 && e_deg < 0.0) || (e_prev < 0.0 && e_deg > 0.0);
      if (crossed_zero && std::abs(e_deg) <= capture_pos_deg_)
      {
        latched_[i] = true;
        latch_timer_[i] = 0.0;
        commands[i].goal_current_mA  = 0.0;
        commands[i].current_limit_mA = current_limit_mA_;
        e_prev_deg_[i] = e_deg;
        continue;
      }

      // ---- Near-zero latch window ----
      const bool near_zero = (std::abs(e_deg) <= latch_pos_deg_) && (std::abs(vel_rad_s) <= latch_vel_rad_s_);
      if (near_zero)
      {
        latched_[i] = true;
        latch_timer_[i] = 0.0;
        commands[i].goal_current_mA  = 0.0;
        commands[i].current_limit_mA = current_limit_mA_;
        e_prev_deg_[i] = e_deg;
        continue;
      }

      // ---- RETURN control (PD in current) ----
      const double tau_cmd = (-(K_return_ * e_rad) - (B_return_ * vel_rad_s));
      double I_cmd_mA = (tau_cmd / Kt_) * 1000.0;

      I_cmd_mA = clamp(I_cmd_mA, -current_limit_mA_, current_limit_mA_);

      commands[i].goal_current_mA  = I_cmd_mA;
      commands[i].current_limit_mA = current_limit_mA_;

      e_prev_deg_[i] = e_deg;
    }
  }

private:
  std::size_t n_;

  std::vector<double> position_ref_deg_;
  std::vector<double> e_prev_deg_;

  // latch states
  std::vector<bool>   latched_;
  std::vector<double> latch_timer_;

  // RETURN gains
  double K_return_ = 0.35;   // Nm/rad  
  double B_return_ = 0.25;   // Nm/(rad/s) 

  double dt_ = 0.02;

  // motor constant
  double Kt_ = 0.354; // Nm/A

  // safety
  double current_limit_mA_ = 180.0;

  // PUSH detect params
  double vel_push_rad_s_ = 0.50; 
  double e_grow_deg_ = 0.15; 

  // LATCH params
  double latch_pos_deg_ = 1.0;
  double latch_vel_rad_s_ = 0.25;
  double capture_pos_deg_ = 1.5;

  double unlatch_pos_deg_ = 3.0; 
  double latch_hold_time_ = 0.10;

  double c_ext_dummy_ = 0.0;

  static double deg2rad(double deg) { return deg * M_PI / 180.0; }

  static double clamp(double x, double lo, double hi)
  {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
  }
};
