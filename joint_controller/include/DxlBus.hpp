#pragma once

#include "MotorTypes.hpp"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <cmath>

class DxlBus
{
public:
  DxlBus(std::string device, uint32_t baud, std::vector<uint8_t> ids)
    : device_(std::move(device)), baud_(baud), ids_(std::move(ids)) {}

  bool begin()
  {
    if (!wb_.begin(device_.c_str(), baud_))
    { ROS_ERROR("Failed to open %s @ %u", device_.c_str(), baud_); return false; }

    for (auto id 
      .: ids_)
      if (!wb_.ping(id)) { ROS_ERROR("Ping failed for ID %u", (unsigned)id); return false; }

    const char* log = nullptr;

    // ===== SyncWrite: Goal_Current =====
    if (!wb_.addSyncWriteHandler(ids_.front(), "Goal_Current", &log))
    { ROS_ERROR("addSyncWriteHandler(Goal_Current) failed: %s", log ? log : ""); return false; }

    // ===== SyncRead: Present_Position / Present_Velocity / Present_Current =====
    if (!wb_.addSyncReadHandler(ids_.front(), "Present_Position", &log))
    { ROS_ERROR("addSyncReadHandler(Present_Position) failed: %s", log ? log : ""); return false; }

    if (!wb_.addSyncReadHandler(ids_.front(), "Present_Velocity", &log))
    { ROS_ERROR("addSyncReadHandler(Present_Velocity) failed: %s", log ? log : ""); return false; }

    if (!wb_.addSyncReadHandler(ids_.front(), "Present_Current", &log))
    { ROS_ERROR("addSyncReadHandler(Present_Current) failed: %s", log ? log : ""); return false; }

    // Handler indices follow the order we added above
    write_handler_    = 0; // Goal_Current
    read_pos_handler_ = 0; // Present_Position
    read_vel_handler_ = 1; // Present_Velocity
    read_cur_handler_ = 2; // Present_Current

    const std::size_t n = ids_.size();
    raw_goal_cur_.assign(n, 0);
    raw_pos_.assign(n, 0);
    raw_vel_.assign(n, 0);
    raw_cur_.assign(n, 0);
    return true;
  }

  // kp/ki/kd are not used in current mode, keep signature for compatibility
  bool configureMotors(int /*kp*/, int /*ki*/, int /*kd*/)
  {
    for (auto id : ids_)
    {
      wb_.torqueOff(id);

      // ===== Current Control Mode =====
      // 0: Current Control
      wb_.itemWrite(id, "Operating_Mode", 0);

      wb_.torqueOn(id);
      int32_t mode = -1;
      if (wb_.itemRead(id, "Operating_Mode", &mode))
        ROS_INFO("[DxlBus] ID %u Operating_Mode = %d (expect 0 for Current Control)", (unsigned)id, (int)mode);
      else
        ROS_WARN("[DxlBus] ID %u failed to read Operating_Mode", (unsigned)id);
    }
    return true;
  }

  bool readAll(std::vector<MotorState>& states)
  {
    if (states.size() != ids_.size()) states.assign(ids_.size(), MotorState{});
    const char* log = nullptr;
    bool ok = true;

    ok &= wb_.syncRead(read_pos_handler_, &log);
    ok &= wb_.getSyncReadData(read_pos_handler_, raw_pos_.data(), &log);

    ok &= wb_.syncRead(read_vel_handler_, &log);
    ok &= wb_.getSyncReadData(read_vel_handler_, raw_vel_.data(), &log);

    ok &= wb_.syncRead(read_cur_handler_, &log);
    ok &= wb_.getSyncReadData(read_cur_handler_, raw_cur_.data(), &log);

    for (std::size_t i = 0; i < ids_.size(); ++i)
    {
      const uint8_t id = ids_[i];

      // Position: raw -> rad -> deg
      const double pos_rad = wb_.convertValue2Radian(id, raw_pos_[i]);
      states[i].position_deg = pos_rad * 180.0 / M_PI;

      // Velocity: raw -> (toolbox conversion) -> rad/s
      // If your toolbox returns "rev/min" or something else, this is the only line to adjust.
      states[i].velocity = wb_.convertValue2Velocity(id, raw_vel_[i]);

      // Current: mA
      states[i].current_mA = wb_.convertValue2Current(id, raw_cur_[i]);

      states[i].comm_ok = ok;
    }

    if (!ok) ROS_WARN_THROTTLE(1.0, "DxlBus readAll() comm errors: %s", log ? log : "");
    return ok;
  }

  bool writeAll(const std::vector<MotorCommand>& commands)
  {
    if (commands.size() != ids_.size()) return false;

    for (std::size_t i = 0; i < ids_.size(); ++i)
    {
      const uint8_t id = ids_[i];

      // clamp safety
      double mA = commands[i].goal_current_mA;
      if (mA >  commands[i].current_limit_mA) mA =  commands[i].current_limit_mA;
      if (mA < -commands[i].current_limit_mA) mA = -commands[i].current_limit_mA;

      raw_goal_cur_[i] = wb_.convertCurrent2Value(id, (float)mA);
    }

    const char* log = nullptr;
    const bool ok = wb_.syncWrite(write_handler_, raw_goal_cur_.data(), &log);
    if (!ok) ROS_WARN_THROTTLE(1.0, "DxlBus writeAll() failed: %s", log ? log : "");
    return ok;
  }

private:
  std::string device_;
  uint32_t baud_;
  std::vector<uint8_t> ids_;
  DynamixelWorkbench wb_;

  uint8_t write_handler_    = 0;
  uint8_t read_pos_handler_ = 0;
  uint8_t read_vel_handler_ = 0;
  uint8_t read_cur_handler_ = 0;

  std::vector<int32_t> raw_goal_cur_, raw_pos_, raw_vel_, raw_cur_;
};
