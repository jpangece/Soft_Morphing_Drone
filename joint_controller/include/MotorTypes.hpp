#pragma once

#include <cstdint>

struct MotorState
{
  double position_deg = 0.0;
  double velocity     = 0.0;
  double current_mA   = 0.0;
  bool   comm_ok      = true;
};

struct MotorCommand
{
  // Current control command (mA)
  double goal_current_mA   = 0.0;

  // Safety limit (mA)
  double current_limit_mA  = 200.0;
};
