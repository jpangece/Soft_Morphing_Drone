#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include "std_msgs/Float64.h"
#include "Tools/InitParam.hpp"
#include <vector>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dynamixel_publisher");
    ros::NodeHandle nh;

    int motor_count;
    nh.param("motor/count", motor_count, 2);

    std::vector<ros::Publisher> motor_pubs;
    motor_pubs.reserve(motor_count);

    for (int i = 0; i < motor_count; ++i)
    {
        std::string topic_name = "/motor_" + std::to_string(i + 1) + "/goal_position";
        ros::Publisher pub = nh.advertise<std_msgs::Float64>(topic_name, 10);
        motor_pubs.push_back(pub);
        ROS_INFO("Created publisher: %s", topic_name.c_str());
    }
    ros::Rate loop_rate(500);

    while (ros::ok())
    {
        iniLoad("/home/lanny/dy_test_ws/src/joint_controller/src/publish_joint_angle.ini");
        for (int i = 0; i < motor_count; ++i)
        {
            std::string key = "goal_position_" + std::to_string(i + 1);
            double goal_pos = iniGet<double>(key, 0.0);

            std_msgs::Float64 msg;
            msg.data = goal_pos;

            motor_pubs[i].publish(msg);

            ROS_INFO("motor_%d goal: %.3f", i + 1, goal_pos);
        }

        loop_rate.sleep();
    }
    return 0;
}
