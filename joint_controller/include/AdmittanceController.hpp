#pragma once

#include "MotorTypes.hpp"
#include <vector>
#include <cmath>

class AdmittanceController
{
public:
  explicit AdmittanceController(std::size_t n)
    : n_(n),
      delta_theta_deg_(n, 0.0),
      delta_theta_d_deg_(n, 0.0),
      tau_ext_f_(n, 0.0),
      position_ref_deg_(n, 0.0)
  {}

  void setParam(double M, double B, double K) { M_ = M; B_ = B; K_ = K; }
  void setDt(double dt) { dt_ = dt; }
  void setThreshold(double tau_threshold, double release_threshold)
  { tau_threshold_ = tau_threshold; release_threshold_ = release_threshold; }

  void setPositionRefDeg(const std::vector<double>& ref)
  { if (ref.size() == n_) position_ref_deg_ = ref; }

  void update(const std::vector<MotorState>& states, std::vector<MotorCommand>& commands)
  {
    if (states.size() != n_ || commands.size() != n_) return;

    for (std::size_t i = 0; i < n_; ++i)
    {
      double goal = position_ref_deg_[i];

      const double tau_ext = -states[i].current_mA * mA_to_A_ * Kt_;
      tau_ext_f_[i] = alpha_ * tau_ext_f_[i] + (1.0 - alpha_) * tau_ext;
      const double tau_abs = std::abs(tau_ext_f_[i]);

      double delta_rad   = deg2rad(delta_theta_deg_[i]);
      double delta_d_rad = deg2rad(delta_theta_d_deg_[i]);

      if (tau_abs <= release_threshold_)
      {
        delta_d_rad = 0.0;
        delta_rad *= decay_factor_1_;
      }
      else if (tau_abs >= tau_threshold_)
      {
        const double tau_eff = tau_ext_f_[i] - sign(tau_ext_f_[i]) * tau_threshold_;
        const double delta_dd = (tau_eff - B_ * delta_d_rad - K_ * delta_rad) / M_;
        delta_d_rad += dt_ * delta_dd;
        delta_rad   += dt_ * delta_d_rad;
      }
      else
      {
        delta_d_rad = 0.0;
        delta_rad *= decay_factor_2_;
      }

      delta_theta_d_deg_[i] = rad2deg(delta_d_rad);
      delta_theta_deg_[i]   = rad2deg(delta_rad);

      commands[i].goal_position_deg = goal + delta_theta_deg_[i];
    }
  }

private:
  std::size_t n_;

  double M_ = 0.05, B_ = 0.5, K_ = 2.0, dt_ = 0.02;
  double alpha_ = 0.8, Kt_ = 0.354, mA_to_A_ = 0.001;
  double tau_threshold_ = 0.04, release_threshold_ = 0.008;
  double decay_factor_1_ = 0.9, decay_factor_2_ = 0.98;

  std::vector<double> delta_theta_deg_, delta_theta_d_deg_, tau_ext_f_, position_ref_deg_;

  static double deg2rad(double deg) { return deg * M_PI / 180.0; }
  static double rad2deg(double rad) { return rad * 180.0 / M_PI; }
  static double sign(double x) { return (x > 0) - (x < 0); }
};
