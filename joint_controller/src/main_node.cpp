#include "DxlBus.hpp"
#include "AdmittanceController.hpp"

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "joint_controller/MotorState.h"

#include <vector>
#include <string>

static std::vector<double> g_position_ref_deg;
static bool g_ref_received = false;

static void refCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  g_position_ref_deg = msg->data;
  g_ref_received = true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "multi_motor_control_sync");
  ros::NodeHandle nh;

  std::string device;
  int baud, kp, ki, kd;
  double M, B, K;
  int node_rate, admittance_enable;

  nh.param<std::string>("motor/device", device, std::string("/dev/ttyUSB0"));
  nh.param("motor/baud", baud, 3000000);
  nh.param("motor/kp", kp, 400);
  nh.param("motor/ki", ki, 0);
  nh.param("motor/kd", kd, 0);

  nh.param("controller/M", M, 0.05);
  nh.param("controller/B", B, 0.5);
  nh.param("controller/K", K, 2.0);
  nh.param("controller/enable", admittance_enable, 1);

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

  std::vector<MotorState> states(n);
  std::vector<MotorCommand> commands(n);

  bus.readAll(states);
  g_position_ref_deg.resize(n);
  for (std::size_t i = 0; i < n; ++ i) g_position_ref_deg[i] = 0.0;
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
      msg.current = states[i].current_mA;
      state_pubs[i].publish(msg);
    }

    if (g_ref_received && g_position_ref_deg.size() == n)
    {
      controller.setPositionRefDeg(g_position_ref_deg);
      g_ref_received = false;
    }

    if (admittance_enable) controller.update(states, commands);
    else
      for (std::size_t i = 0; i < n; ++i) commands[i].goal_position_deg = g_position_ref_deg[i];

    bus.writeAll(commands);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
