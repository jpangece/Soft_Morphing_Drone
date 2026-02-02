#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <ros/ros.h>
#include "joint_controller/MotorState.h"

class DxlMotor
{
public:
    DxlMotor(uint8_t id, DynamixelWorkbench &wb, ros::NodeHandle &nh)
        : id_(id), wb_(wb), nh_(nh), pos_(0.0), vel_(0.0), current_(0.0)
    {
        pub_ = nh_.advertise<joint_controller::MotorState>("motor_" + std::to_string(id_) + "/state", 10);
        // 不要在 callback 里做 itemRead/itemWrite；总线 I/O 全部放主循环 syncRead/syncWrite
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
        return flag1 && flag2 && flag3;
    }

    // 主节点用 syncRead 读到 raw 后转换，再把值塞进来
    void setState(double position_deg, double velocity, double current_mA)
    {
        pos_ = position_deg;
        vel_ = velocity;
        current_ = current_mA;
    }

    void pubStatus()
    {
        joint_controller::MotorState msg;
        msg.position = pos_;
        msg.velocity = vel_;
        msg.current = current_;
        pub_.publish(msg);
    }

    uint8_t getID() const { return id_; }

private:
    uint8_t id_;
    DynamixelWorkbench &wb_;
    ros::NodeHandle &nh_;
    ros::Publisher pub_;

    float pos_;
    float vel_;
    float current_;
};
