#include "DxlBus.hpp"
#include "AdmittanceController.hpp"

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "joint_controller/MotorState.h"

#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <algorithm>

static std::vector<double> g_position_ref_deg;
static bool g_ref_received = false;

static void refCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  g_position_ref_deg = msg->data;
  g_ref_received = true;
}

static inline double clamp(double x, double lo, double hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "multi_motor_control_sync");
  ros::NodeHandle nh;

  std::string device;
  int baud, kp, ki, kd;
  double M, B, K;
  int node_rate, admittance_enable;

  // controller params (optional)
  double current_limit_mA, Kt, alpha, c_ext, tau_th, release_th;

  nh.param<std::string>("motor/device", device, std::string("/dev/ttyUSB0"));
  nh.param("motor/baud", baud, 3000000);

  // kept for compatibility; DxlBus current mode doesn't use these gains
  nh.param("motor/kp", kp, 400);
  nh.param("motor/ki", ki, 0);
  nh.param("motor/kd", kd, 0);

  nh.param("controller/M", M, 0.05);
  nh.param("controller/B", B, 0.02);
  nh.param("controller/K", K, 0.25);
  nh.param("controller/enable", admittance_enable, 1);

  nh.param("controller/current_limit_mA", current_limit_mA, 200.0);
  nh.param("controller/Kt", Kt, 0.354);
  nh.param("controller/alpha", alpha, 0.8);
  nh.param("controller/c_ext", c_ext, 0.5);
  nh.param("controller/tau_threshold", tau_th, 0.04);
  nh.param("controller/release_threshold", release_th, 0.008);

  nh.param("node_rate", node_rate, 50);

  // IDs
  std::vector<int> ids_i;
  if (!nh.getParam("motor/ids", ids_i))
  {
    int motor_count;
    nh.param("motor/count", motor_count, 4);
    ids_i.resize(motor_count);
    for (int i = 0; i < motor_count; ++i) ids_i[i] = i + 1;  // typical Dynamixel IDs start from 1
  }

  std::vector<uint8_t> ids;
  ids.reserve(ids_i.size());
  for (int v : ids_i) ids.push_back(static_cast<uint8_t>(v));

  const std::size_t n = ids.size();
  const double dt = 1.0 / static_cast<double>(node_rate);

  // Publishers: states
  std::vector<ros::Publisher> state_pubs;
  state_pubs.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto topic = "/motor_" + std::to_string(ids[i]) + "/state";
    state_pubs.push_back(nh.advertise<joint_controller::MotorState>(topic, 10));
  }

  // Publisher: command currents (for visual checking)
  ros::Publisher cmd_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/motor/goal_current_mA", 10);

  // Subscriber: reference positions
  ros::Subscriber ref_sub =
      nh.subscribe<std_msgs::Float64MultiArray>("/motor/position_ref_deg", 10, refCallback);

  // Bus + controller
  DxlBus bus(device, static_cast<uint32_t>(baud), ids);
  if (!bus.begin()) return 1;
  bus.configureMotors(kp, ki, kd);

  AdmittanceController controller(n);
  controller.setParam(M, B, K);
  controller.setDt(dt);
  controller.setCurrentLimit(current_limit_mA);
  controller.setKt(Kt);
  controller.setAlpha(alpha);
  controller.setExternalComp(c_ext);
  controller.setThreshold(tau_th, release_th);

  std::vector<MotorState> states(n);
  std::vector<MotorCommand> commands(n);

  // initial ref: zeros
  bus.readAll(states);
  g_position_ref_deg.assign(n, 0.0);
  controller.setPositionRefDeg(g_position_ref_deg);

  const double DEG2RAD = M_PI / 180.0;

  // Debug timer
  ros::Time last_dbg = ros::Time::now();

  ros::Rate rate(node_rate);
  while (ros::ok())
  {
    bus.readAll(states);

    // publish per-motor state topics
    for (std::size_t i = 0; i < n; ++i)
    {
      joint_controller::MotorState msg;
      msg.position = states[i].position_deg;
      msg.velocity = states[i].velocity;
      msg.current  = states[i].current_mA;
      state_pubs[i].publish(msg);
    }

    // update ref if received
    if (g_ref_received && g_position_ref_deg.size() == n)
    {
      controller.setPositionRefDeg(g_position_ref_deg);
      g_ref_received = false;
    }

    // compute commands
    if (admittance_enable)
    {
      controller.update(states, commands);
    }
    else
    {
      // fallback: soft spring-to-ref in current space (keeps behavior sane)
      for (std::size_t i = 0; i < n; ++i)
      {
        const double ref_deg = g_position_ref_deg[i];
        const double e_rad = (states[i].position_deg - ref_deg) * DEG2RAD;
        const double vel_rad_s = states[i].velocity;

        const double tau_cmd = (-(K * e_rad) - (B * vel_rad_s));
        double I_cmd_mA = (tau_cmd / Kt) * 1000.0;
        I_cmd_mA = clamp(I_cmd_mA, -current_limit_mA, current_limit_mA);

        commands[i].goal_current_mA  = I_cmd_mA;
        commands[i].current_limit_mA = current_limit_mA;
      }
    }

    // publish command currents as one array
    std_msgs::Float64MultiArray cmd_msg;
    cmd_msg.data.resize(n);
    for (std::size_t i = 0; i < n; ++i) cmd_msg.data[i] = commands[i].goal_current_mA;
    cmd_pub.publish(cmd_msg);

    // ===== 1 Hz DEBUG PRINT (both ROS + stderr) =====
    ros::Time now = ros::Time::now();
    if ((now - last_dbg).toSec() >= 1.0)
    {
      last_dbg = now;

      // Summaries
      double max_abs_cmd = 0.0, max_abs_cur = 0.0;
      for (std::size_t i = 0; i < n; ++i)
      {
        max_abs_cmd = std::max(max_abs_cmd, std::abs(commands[i].goal_current_mA));
        max_abs_cur = std::max(max_abs_cur, std::abs(states[i].current_mA));
      }

      const double ref0 = (n > 0 ? g_position_ref_deg[0] : 0.0);
      const double pos0 = (n > 0 ? states[0].position_deg : 0.0);
      const double vel0 = (n > 0 ? states[0].velocity : 0.0);
      const double cur0 = (n > 0 ? states[0].current_mA : 0.0);
      const double cmd0 = (n > 0 ? commands[0].goal_current_mA : 0.0);

      ROS_WARN("[DBG] ref0=%.1f deg pos0=%.1f deg vel0=%.3f rad/s cur0=%.1f mA cmd0=%.1f mA | max|Cur|=%.1f mA max|Cmd|=%.1f mA | enable=%d",
               ref0, pos0, vel0, cur0, cmd0, max_abs_cur, max_abs_cmd, admittance_enable);

      // Always visible even if ROS console settings are weird
      std::fprintf(stderr,
                   "[DBG] ref0=%.1f pos0=%.1f vel0=%.3f cur0=%.1f cmd0=%.1f | max|Cur|=%.1f max|Cmd|=%.1f | enable=%d\n",
                   ref0, pos0, vel0, cur0, cmd0, max_abs_cur, max_abs_cmd, admittance_enable);
      std::fflush(stderr);
    }

    // write to motors
    bus.writeAll(commands);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
