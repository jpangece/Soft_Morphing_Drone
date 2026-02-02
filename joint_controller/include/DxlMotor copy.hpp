#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "joint_controller/MotorState.h"
#include "Tools/UavMath.hpp"

#define dxl_position_offset 2048

class DxlMotor
{
public:
    DxlMotor(uint8_t id, DynamixelWorkbench &wb, ros::NodeHandle &nh)
        : id_(id), wb_(wb), nh_(nh), pos_(0.0), vel_(0.0), current_(0.0)
    {
        pub_ = nh_.advertise<joint_controller::MotorState>("motor_" + std::to_string(id_) + "/state", 10);
        sub_ = nh_.subscribe("motor_" + std::to_string(id_) + "/goal_position", 10, &DxlMotor::goalCallback, this);
    }

    bool init()
    {
        bool result = wb_.ping(id_);
        if (!result)
        {
            ROS_ERROR("Failed to ping motor ID %d", id_);
            return false;
        }
        wb_.torqueOff(id_);
        wb_.itemWrite(id_, "Operating_Mode", 3); // position mode
        wb_.torqueOn(id_);
        return true;
    }

    bool setPID(double kp, double ki, double kd)
    {
        bool flag1 = wb_.itemWrite(id_, "Position_P_Gain", kp);
        bool flag2 = wb_.itemWrite(id_, "Position_I_Gain", ki);
        bool flag3 = wb_.itemWrite(id_, "Position_D_Gain", kd);
        if (flag1 && flag2 && flag3)
            return true;
        return false;
    }

    void readState()
    {
        int32_t raw_pos, raw_vel, raw_cur;
        wb_.itemRead(id_, "Present_Position", &raw_pos);
        wb_.itemRead(id_, "Present_Velocity", &raw_vel);
        wb_.itemRead(id_, "Present_Current", &raw_cur);

        pos_ = wb_.convertValue2Radian(id_, raw_pos);
        pos_ = rad2deg(pos_);
        vel_ = wb_.convertValue2Velocity(id_, raw_vel);
        current_ = wb_.convertValue2Current(id_, raw_cur);
    }

    void writePositionDeg(const float position_deg)
    {
        int32_t goal = get_dxl_position(position_deg);
        wb_.itemWrite(id_, "Goal_Position", goal);
    }

    void pubStatus()
    {
        joint_controller::MotorState msg;
        msg.position = pos_;
        msg.velocity = vel_;
        msg.current = current_;
        pub_.publish(msg);
    }

    float getPosition() const { return pos_; }
    float getVelocity() const { return vel_; }
    float getCurrent() const { return current_; }
    uint8_t getID() const { return id_; }

private:
    uint8_t id_;
    DynamixelWorkbench &wb_;
    ros::NodeHandle &nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

    float pos_;
    float vel_;
    float current_;
    int get_dxl_position(const float position_deg)
    {
        int goal_position_dxl;
        goal_position_dxl = round(position_deg * 1024 / 90) + dxl_position_offset;
        return goal_position_dxl;
    }
    void goalCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        writePositionDeg(msg->data);
    }
};
