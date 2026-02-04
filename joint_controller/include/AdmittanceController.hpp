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
      I_cmd_f_mA_(n, 0.0)
  {}

  // NOTE:
  // M, B, K in Current-control based:
  // K_ : Virtual Spring (Nm/rad)
  // B_ : Virtual Damping (Nm/(rad/s))
  // M_ : not used currently

  void setParam(double M, double B, double K) { M_ = M; B_ = B; K_ = K; }
  void setDt(double dt) { dt_ = dt; }

  void setThreshold(double tau_threshold, double release_threshold)
  { tau_threshold_ = tau_threshold; release_threshold_ = release_threshold; }

  void setPositionRefDeg(const std::vector<double>& ref)
  { if (ref.size() == n_) position_ref_deg_ = ref; }

  // current safety
  void setCurrentLimit(double limit_mA) { current_limit_mA_ = limit_mA; }

  // motor torque constant (Nm/A)
  void setKt(double Kt) { Kt_ = Kt; }

  // low-pass filter alpha for external torque estimate
  void setAlpha(double alpha) { alpha_ = clamp(alpha, 0.0, 0.999); }

  // external torque compensation [0,1]
  void setExternalComp(double c_ext) { c_ext_ = clamp(c_ext, 0.0, 1.0); }

  // deadband around reference to eliminate limit-cycle
  void setDeadband(double pos_deadband_deg, double vel_deadband_rad_s)
  {
    pos_deadband_deg_ = std::max(0.0, pos_deadband_deg);
    vel_deadband_rad_s_ = std::max(0.0, vel_deadband_rad_s);
  }

  void setCmdFilterAlpha(double a) { cmd_alpha_ = clamp(a, 0.0, 0.999); }

  // output: commands[i].goal_current_mA
  void update(const std::vector<MotorState>& states, std::vector<MotorCommand>& commands)
  {
    if (states.size() != n_ || commands.size() != n_) return;

    for (std::size_t i = 0; i < n_; ++i)
    {
      const double ref_deg = position_ref_deg_[i];
      const double pos_deg = states[i].position_deg;
      const double vel_rad_s = states[i].velocity;

      // error
      const double e_deg = pos_deg - ref_deg;
      const double e_rad = deg2rad(e_deg);

      // ===== Hard deadband near reference =====
      if (std::abs(e_deg) <= pos_deadband_deg_ && std::abs(vel_rad_s) <= vel_deadband_rad_s_)
      {
        I_cmd_f_mA_[i] = 0.0;
        commands[i].goal_current_mA  = 0.0;
        commands[i].current_limit_mA = current_limit_mA_;
        continue;
      }

      // ===== external torque estimate from current (Nm) =====
      const double tau_ext = -states[i].current_mA * mA_to_A_ * Kt_;
      tau_ext_f_[i] = alpha_ * tau_ext_f_[i] + (1.0 - alpha_) * tau_ext;

      const double tau_abs = std::abs(tau_ext_f_[i]);
      double tau_ext_used = 0.0;

      if (tau_abs >= tau_threshold_)
      {
        tau_ext_used = tau_ext_f_[i] - sign(tau_ext_f_[i]) * tau_threshold_;
      }
      else if (tau_abs <= release_threshold_)
      {
        tau_ext_used = 0.0;
      }
      else
      {
        tau_ext_used = 0.0;
      }

      // ===== virtual impedance to ref =====
      const double tau_cmd = (-(K_ * e_rad) - (B_ * vel_rad_s)) - (c_ext_ * tau_ext_used);

      // torque -> current (mA)
      double I_cmd_mA = (tau_cmd / Kt_) * 1000.0;

      // clamp
      I_cmd_mA = clamp(I_cmd_mA, -current_limit_mA_, current_limit_mA_);

      // ===== LPF command to avoid oscillation from quantization/noise =====
      I_cmd_f_mA_[i] = cmd_alpha_ * I_cmd_f_mA_[i] + (1.0 - cmd_alpha_) * I_cmd_mA;

      commands[i].goal_current_mA  = I_cmd_f_mA_[i];
      commands[i].current_limit_mA = current_limit_mA_;
    }
  }

private:
  std::size_t n_;

  double M_ = 0.05;
  double B_ = 0.04;     
  double K_ = 0.20;
  double dt_ = 0.02;

  double alpha_ = 0.8;
  double Kt_ = 0.354;
  double mA_to_A_ = 0.001;

  double tau_threshold_ = 0.04;
  double release_threshold_ = 0.010;

  double c_ext_ = 0.2;

  double current_limit_mA_ = 200.0;

  // NEW deadband
  double pos_deadband_deg_ = 1.0; 
  double vel_deadband_rad_s_ = 0.15;

  // NEW command LPF
  double cmd_alpha_ = 0.7;

  std::vector<double> tau_ext_f_;
  std::vector<double> position_ref_deg_;
  std::vector<double> I_cmd_f_mA_;

  static double deg2rad(double deg) { return deg * M_PI / 180.0; }
  static double sign(double x) { return (x > 0) - (x < 0); }

  static double clamp(double x, double lo, double hi)
  {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
  }
};
