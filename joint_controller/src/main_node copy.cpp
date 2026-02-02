#include "DxlMotor.hpp"
#include "AdmittanceController.hpp"
#include <ros/ros.h>
#include <vector>
#include "Tools/InitParam.hpp"

#define DEVICENAME "/dev/ttyUSB0"
#define BAUDRATE 57600

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "multi_motor_control");
    ros::NodeHandle nh;

    int kp, ki, kd;
    double M, B, K;
    int admittance_mode_enable;
    int motor_count, motor_init_pos;
    int node_rate;

    nh.param("motor/kp", kp, 400);
    nh.param("motor/ki", ki, 0);
    nh.param("motor/kd", kd, 0);

    nh.param("controller/M", M, 0.05);
    nh.param("controller/B", B, 0.5);
    nh.param("controller/K", K, 2.0);
    nh.param("controller/enable", admittance_mode_enable, 1);

    nh.param("motor/count", motor_count, 2);
    nh.param("motor/init_pos", motor_init_pos, 0);
    nh.param("node_rate", node_rate, 50);

    ROS_INFO("kp: %d, ki: %d, kd: %d", kp, ki, kd);

    ros::Rate rate(node_rate);
    double dt = 1 / node_rate;

    DynamixelWorkbench wb;
    wb.begin(DEVICENAME, BAUDRATE);

    AdmittanceController controller(nh, motor_count);
    controller.setParam(M, B, K);

    std::vector<std::shared_ptr<DxlMotor>> motors;

    for (int i = 0; i < motor_count; ++i)
    {
        int motor_id = i; // make sure id starts from 1
        auto motor = std::make_shared<DxlMotor>(motor_id, wb, nh);
        if (!motor->init())
        {
            ROS_WARN("Failed to init motor ID %d", motor_id);
        }
        motor->setPID(kp, ki, kd);
        motor->writePositionDeg(motor_init_pos);
        motors.push_back(motor);
    }

    while (ros::ok())
    {
        for (auto &m : motors)
        {
            m->readState();
            m->pubStatus();
        }
        if (admittance_mode_enable)
            controller.update();

        ros::spinOnce();
        rate.sleep();
    }
}
