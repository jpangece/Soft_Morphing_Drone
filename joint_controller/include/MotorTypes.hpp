#pragma once

#include <cstdint>
#include <vector>

struct MotorState
{
  double position_deg = 0.0;
  double velocity = 0.0;   // optional
  double current_mA = 0.0;
  bool   comm_ok = true;
};

struct MotorCommand
{
  double goal_position_deg = 0.0;
};
