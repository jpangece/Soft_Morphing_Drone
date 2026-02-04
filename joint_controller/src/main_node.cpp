#include "DxlBus.hpp"
#include "AdmittanceController.hpp"

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "joint_controller/MotorState.h"

#include <vector>
#include <string>
#include <cmath>
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

  // controller extra params
  double current_limit_mA, Kt, alpha, c_ext, tau_th, release_th;

  nh.param<std::string>("motor/device", device, std::string("/dev/ttyUSB0"));
  nh.param("motor/baud", baud, 3000000);

  // kept for compatibility
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

  std::vector<int> ids_i;
  if (!nh.getParam("motor/ids", ids_i))
  {
    int motor_count;
    nh.param("motor/count", motor_count, 4);
    ids_i.resize(motor_count);
    for (int i = 0; i < motor_count; ++i) ids_i[i] = i;
  }

  std::vector<uint8_t> ids;
  ids.reserve(ids_i.size());
  for (int v : ids_i) ids.push_back(static_cast<uint8_t>(v));

  const std::size_t n = ids.size();
  const double dt = 1.0 / static_cast<double>(node_rate);
  const double DEG2RAD = M_PI / 180.0;

  std::vector<ros::Publisher> state_pubs;
  state_pubs.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto topic = "/motor_" + std::to_string(ids[i]) + "/state";
    state_pubs.push_back(nh.advertise<joint_controller::MotorState>(topic, 10));
  }

  ros::Subscriber ref_sub = nh.subscribe<std_msgs::Float64MultiArray>(
      "/motor/position_ref_deg", 10, refCallback);

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

  bus.readAll(states);

  g_position_ref_deg.assign(n, 0.0);
  controller.setPositionRefDeg(g_position_ref_deg);

  ros::Rate rate(node_rate);
  while (ros::ok())
  {
    bus.readAll(states);

    for (std::size_t i = 0; i < n; ++i)
    {
      joint_controller::MotorState msg;
      msg.position = states[i].position_deg;
      msg.velocity = states[i].velocity;
      msg.current  = states[i].current_mA;
      state_pubs[i].publish(msg);
    }

    if (g_ref_received && g_position_ref_deg.size() == n)
    {
      controller.setPositionRefDeg(g_position_ref_deg);
      g_ref_received = false;
    }

    if (admittance_enable)
    {
      controller.update(states, commands);
    }
    else
    {
      // fallback: soft spring-to-ref in current space
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

    bus.writeAll(commands);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
