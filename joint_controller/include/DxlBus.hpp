#pragma once

#include "MotorTypes.hpp"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <ros/ros.h>
#include <vector>
#include <string>

class DxlBus
{
public:
  DxlBus(std::string device, uint32_t baud, std::vector<uint8_t> ids)
    : device_(std::move(device)), baud_(baud), ids_(std::move(ids)) {}

  bool begin()
  {
    if (!wb_.begin(device_.c_str(), baud_))
    { ROS_ERROR("Failed to open %s @ %u", device_.c_str(), baud_); return false; }

    for (auto id : ids_)
      if (!wb_.ping(id)) { ROS_ERROR("Ping failed for ID %u", (unsigned)id); return false; }

    const char* log = nullptr;

    if (!wb_.addSyncWriteHandler(ids_.front(), "Goal_Position", &log))
    { ROS_ERROR("addSyncWriteHandler(Goal_Position) failed: %s", log ? log : ""); return false; }

    if (!wb_.addSyncReadHandler(ids_.front(), "Present_Position", &log))
    { ROS_ERROR("addSyncReadHandler(Present_Position) failed: %s", log ? log : ""); return false; }

    if (!wb_.addSyncReadHandler(ids_.front(), "Present_Current", &log))
    { ROS_ERROR("addSyncReadHandler(Present_Current) failed: %s", log ? log : ""); return false; }

    write_handler_ = 0;
    read_pos_handler_ = 0;
    read_cur_handler_ = 1;

    const std::size_t n = ids_.size();
    raw_goal_.assign(n, 0);
    raw_pos_.assign(n, 0);
    raw_cur_.assign(n, 0);
    return true;
  }

  bool configureMotors(int kp, int ki, int kd)
  {
    for (auto id : ids_)
    {
      wb_.torqueOff(id);
      wb_.itemWrite(id, "Operating_Mode", 3);
      wb_.itemWrite(id, "Position_P_Gain", kp);
      wb_.itemWrite(id, "Position_I_Gain", ki);
      wb_.itemWrite(id, "Position_D_Gain", kd);
      wb_.torqueOn(id);
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

    ok &= wb_.syncRead(read_cur_handler_, &log);
    ok &= wb_.getSyncReadData(read_cur_handler_, raw_cur_.data(), &log);

    for (std::size_t i = 0; i < ids_.size(); ++i)
    {
      const uint8_t id = ids_[i];
      const double pos_rad = wb_.convertValue2Radian(id, raw_pos_[i]);
      states[i].position_deg = pos_rad * 180.0 / M_PI;
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
      const double rad = commands[i].goal_position_deg * M_PI / 180.0;
      raw_goal_[i] = wb_.convertRadian2Value(id, rad);
    }

    const char* log = nullptr;
    const bool ok = wb_.syncWrite(write_handler_, raw_goal_.data(), &log);
    if (!ok) ROS_WARN_THROTTLE(1.0, "DxlBus writeAll() failed: %s", log ? log : "");
    return ok;
  }

private:
  std::string device_;
  uint32_t baud_;
  std::vector<uint8_t> ids_;
  DynamixelWorkbench wb_;

  bool enable_velocity_ = false;

  uint8_t write_handler_ = 0;
  uint8_t read_pos_handler_ = 0;
  uint8_t read_cur_handler_ = 0;
  uint8_t read_vel_handler_ = 255;

  std::vector<int32_t> raw_goal_, raw_pos_, raw_cur_, raw_vel_;
};
