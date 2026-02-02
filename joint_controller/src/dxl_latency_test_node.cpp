#include <ros/ros.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <vector>
#include <cmath>
#include "joint_controller/MotorState.h"

static inline double rad2deg(double r) { return r * 180.0 / M_PI; }
static inline double deg2rad(double d) { return d * M_PI / 180.0; }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dxl_latency_test");
  ros::NodeHandle nh("~");

  std::string device;
  int baudrate;
  int rate_hz;

  nh.param<std::string>("device", device, "/dev/ttyUSB0");
  nh.param("baudrate", baudrate, 57600);
  nh.param("rate", rate_hz, 50);

  ros::Rate rate(rate_hz);

  /* ===== Dynamixel IDs ===== */
  const std::vector<uint8_t> ids = {0, 1, 2};
  const size_t N = ids.size();

  std::vector<ros::Publisher> state_pubs;
  state_pubs.reserve(N);
  for (std::size_t i = 0; i < N; ++i)
  {
    const auto topic = "/motor_" + std::to_string(ids[i]) + "/state";
    state_pubs.push_back(nh.advertise<joint_controller::MotorState>(topic, 10));
  } 
  // 把实际数据发布出来，然后看话题的频率，应该是最直观的检查运行频率是否受到串口限制的方式？

  DynamixelWorkbench wb;
  const char* log = nullptr;

  if (!wb.begin(device.c_str(), baudrate))
  {
    ROS_ERROR("Failed to open %s", device.c_str());
    return 1;
  }

  for (auto id : ids)
  {
    if (!wb.ping(id))
    {
      ROS_ERROR("Ping failed for ID %d", id);
      return 1;
    }

    wb.torqueOff(id);
    wb.itemWrite(id, "Operating_Mode", 3);  // position mode
    wb.torqueOn(id);
  }

  /* ===== Sync handlers ===== */
  // 通过ids[0]传的是ids数组的地址，这边wb就能获取舵机的数量
  if (!wb.addSyncWriteHandler(ids[0], "Goal_Position", &log))
  {
    ROS_ERROR("addSyncWriteHandler failed: %s", log ? log : "");
    return 1;
  }

  if (!wb.addSyncReadHandler(ids[0], "Present_Position", &log))
  {
    ROS_ERROR("addSyncReadHandler(Present_Position) failed");
    return 1;
  }

  if (!wb.addSyncReadHandler(ids[0], "Present_Current", &log))
  {
    ROS_ERROR("addSyncReadHandler(Present_Current) failed");
    return 1;
  }
  
  // 单纯是靠顺序命名 
  const uint8_t WRITE_HANDLER = 0;
  const uint8_t READ_POS_HANDLER = 0;
  const uint8_t READ_CUR_HANDLER = 1;

  std::vector<int32_t> raw_pos(N, 0);
  std::vector<int32_t> raw_cur(N, 0);
  std::vector<int32_t> raw_goal(N, 0);

  /* ===== 初始化 goal = 当前角度 ===== */
  wb.syncRead(READ_POS_HANDLER, &log);
  wb.getSyncReadData(READ_POS_HANDLER, raw_pos.data(), &log);

  for (size_t i = 0; i < N; ++i)
  {
    double rad = wb.convertValue2Radian(ids[i], raw_pos[i]);
    raw_goal[i] = wb.convertRadian2Value(ids[i], rad);
  }

  wb.syncWrite(WRITE_HANDLER, raw_goal.data(), &log);

  ROS_INFO("dxl_latency_test started (%d Hz, baud=%d)", rate_hz, baudrate);

  /* ===== 主循环 ===== */
  while (ros::ok())
  {
    ros::Time t_loop_start = ros::Time::now();

    /* ---- Read Position ---- */
    ros::Time t = ros::Time::now();
    wb.syncRead(READ_POS_HANDLER, &log);
    wb.getSyncReadData(READ_POS_HANDLER, raw_pos.data(), &log);
    double t_read_pos = (ros::Time::now() - t).toSec() * 1000.0;

    /* ---- Read Current ---- */
    t = ros::Time::now();
    wb.syncRead(READ_CUR_HANDLER, &log);
    wb.getSyncReadData(READ_CUR_HANDLER, raw_cur.data(), &log);
    double t_read_cur = (ros::Time::now() - t).toSec() * 1000.0;

    for (std::size_t i = 0; i < N; ++i)
    {
      joint_controller::MotorState msg;
      msg.position = raw_pos[i];
      msg.velocity = 0.0;
      msg.current = raw_cur[i];
      state_pubs[i].publish(msg);
    }

    /* ---- Convert + keep goal ---- */
    for (size_t i = 0; i < N; ++i)
    {
      double pos_rad = wb.convertValue2Radian(ids[i], raw_pos[i]);
      raw_goal[i] = wb.convertRadian2Value(ids[i], 0.0);
    }

    /* ---- Write Goal ---- */
    t = ros::Time::now();
    wb.syncWrite(WRITE_HANDLER, raw_goal.data(), &log);
    double t_write = (ros::Time::now() - t).toSec() * 1000.0;

    double t_loop = (ros::Time::now() - t_loop_start).toSec() * 1000.0;

    /* ---- 打印 ---- */
    ROS_INFO_THROTTLE(
      1.0,
      "[timing ms] read_pos=%.2f  read_cur=%.2f  write=%.2f  loop=%.2f | "
      "pos1=%.2f deg cur1=%.1f mA",
      t_read_pos, t_read_cur, t_write, t_loop,
      rad2deg(wb.convertValue2Radian(ids[0], raw_pos[0])),
      wb.convertValue2Current(ids[0], raw_cur[0])
    );

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
