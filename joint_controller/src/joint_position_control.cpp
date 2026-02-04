#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "Tools/UavMath.hpp"
#include "Tools/InitParam.hpp"

#define DEVICENAME "/dev/ttyUSB0"
#define BAUDRATE 57600

#define MIN_POS -180.0
#define MAX_POS 179.0
// -180 --> 0
// 0 --> 2048
// 179 --> 4095


double goal_position_raw = 45.0;
double goal_position_handled = 0.0;
int32_t dxl_goal_position = 0;
int32_t dxl_position_offset = 2048;
int32_t op_mode = -1;

// Compliance control params (tune)
static const double DEG2RAD = 3.141592653589793 / 180.0;

// Spring-damper gains
double K_spring_Nm_per_rad = 0.25;   // how strong it pulls back
double D_damper_Nm_per_rads = 0.02;  // how much it resists speed

// Current limit for safety(mA)
double I_limit_mA = 200.0;

// clamp helper
static inline double clamp(double x, double lo, double hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}


void set_goal_position(const std_msgs::Float64::ConstPtr &msg)
{
    // check receiving msg
    goal_position_raw = msg->data;
    ROS_INFO("get topic raw data:%f", goal_position_raw);
}

double handle_position_range(double position_)
{
    double position_handled = position_;
    // position_handled = position_ % 360;
    while (position_handled < -180)
    {
        position_handled = position_handled + 360;
    }
    while (position_handled > 179)
    {
        position_handled = position_handled - 360;
    }
    ROS_INFO("position after range fixing: %f", position_handled);
    return position_handled;
}

int get_dxl_position(int position_handled)
{
    int dxl_position;
    dxl_position = round(position_handled * 1024 / 90) + dxl_position_offset;
    return dxl_position;
}

int main(int argc, char *argv[])
{
    iniLoad("/home/lanny/dy_test_ws/src/joint_controller/src/im_control_new.ini");
    double M = iniGet<double>("M", 0.1);
    double B = iniGet<double>("B", 0.2);
    double K = iniGet<double>("K", 0.5);
    ROS_INFO("M:%f, B:%f, K:%f", M, B, K);
    double dt = 0.02;
    double Kt = 0.354; // N*m/A
    double mA_to_A = 0.001;
    double alpha = 0.8;

    ros::init(argc, argv, "dynamixel_read_write");
    ros::NodeHandle nh;
    ros::Subscriber sub_goal_position = nh.subscribe("/dynamixel_goal_position", 1, set_goal_position);
    ros::Publisher pub_current_state = nh.advertise<std_msgs::Float32MultiArray>("/dynamixel_current_state", 10);
    ros::Rate loop_rate(50);

    // region workbench init
    DynamixelWorkbench dxl_wb;
    bool ifbegin = dxl_wb.begin(DEVICENAME, BAUDRATE); // XL330 是 Protocol 2.0
    if (ifbegin)
        ROS_INFO("Succeeded to begin");
    bool ifping = dxl_wb.ping(1);
    if (ifping)
    {
        ROS_INFO("Succeeded to ping!!");

        const char *model_name = dxl_wb.getModelName(1);
        ROS_INFO("Model name: %s", model_name);

        int32_t op_mode = 0;
        if (dxl_wb.itemRead(1, "Operating_Mode", &op_mode))
        {
            ROS_INFO("Current operating mode: %d", op_mode);
        }
        else
        {
            ROS_WARN("Failed to read Operating_Mode");
        }
    }

    dxl_wb.torqueOff(1);

    // CURRENT CONTROL MODE
    if (dxl_wb.setCurrentControlMode(1))
    {
        ROS_INFO("Set current control mode (ID=1) success");
    }
    else
    {
        ROS_WARN("Failed to set current control mode (ID=1)");
    }

    // torque on
    if (dxl_wb.torqueOn(1))
    {
        ROS_INFO("Torque On success");
    }
    else
    {
        ROS_WARN("Torque On failed");
    }
    // region end
    bool ifping2 = dxl_wb.ping(2);
    if (ifping2)
    {
        ROS_INFO("Succeeded to ping2!!");

        const char *model_name2 = dxl_wb.getModelName(2);
        ROS_INFO("Model name2: %s", model_name2);

        int32_t op_mode2 = 0;
        if (dxl_wb.itemRead(2, "Operating_Mode", &op_mode2))
        {
            ROS_INFO("Current operating mode2: %d", op_mode2);
        }
        else
        {
            ROS_WARN("Failed to read Operating_Mode");
        }
    }

    dxl_wb.torqueOff(2);

    if (dxl_wb.setCurrentControlMode(2))
    {
        ROS_INFO("Set current control mode (ID=2) success");
    }
    else
    {
        ROS_WARN("Failed to set current control mode (ID=2)");
    }

    // torque on
    if (dxl_wb.torqueOn(2))
    {
        ROS_INFO("Torque On2 success");
    }
    else
    {
        ROS_WARN("Torque On2 failed");
    }

    float present_position = 0.0;
    float dxl_present_position = 0;
    float present_velocity = 0.0;
    int32_t dxl_present_velocity = 0;
    float present_current = 0.0;
    int32_t dxl_present_current = 0;

    float delta_theta = 0;
    float delta_theta_d = 0;
    float delta_theta_dd = 0;
    double tau_ext = 0.0;
    double tau_ext_f = 0.0;

    while (ros::ok())
    {
        // Read present position (deg)
        dxl_wb.getRadian(1, &dxl_present_position);
        present_position = rad2deg(dxl_present_position);

        // Read present velocity (raw -> rad/s)
        dxl_wb.itemRead(1, "Present_Velocity", &dxl_present_velocity);
        present_velocity = dxl_wb.convertValue2Velocity(1, dxl_present_velocity);

        // Virtual spring-damper to 0 deg 
        double theta_rad = present_position * DEG2RAD;
        double omega_rad_s = present_velocity;

        // desired torque (Nm)
        double tau_cmd = (-K_spring_Nm_per_rad * theta_rad) - (D_damper_Nm_per_rads * omega_rad_s);

        // convert to current (A) using motor torque constant
        double I_cmd_A = tau_cmd / Kt;
        double I_cmd_mA = I_cmd_A * 1000.0;
    
        // clamp current
        I_cmd_mA = clamp(I_cmd_mA, -I_limit_mA, I_limit_mA);

        // Convert mA
        int32_t goal_current_value = dxl_wb.convertCurrent2Value(1, (float)I_cmd_mA);

        // Write same current command to both servos
        dxl_wb.itemWrite(1, "Goal_Current", goal_current_value);
        dxl_wb.itemWrite(2, "Goal_Current", goal_current_value);

        ROS_INFO("pos=%.2f deg, vel=%.3f rad/s, tau_cmd=%.4f Nm, I_cmd=%.1f mA",
             present_position, present_velocity, tau_cmd, I_cmd_mA);
    }
    return 0;
}