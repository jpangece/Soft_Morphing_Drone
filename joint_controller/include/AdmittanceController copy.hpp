#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "joint_controller/MotorState.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159
#endif

class AdmittanceController
{
public:
    AdmittanceController(ros::NodeHandle &nh, int motor_count)
        : nh_(nh), motor_count_(motor_count)
    {
        M_ = 0.05;
        B_ = 0.5;
        K_ = 2.0;
        alpha_ = 0.8;
        Kt_ = 0.354;
        mA_to_A_ = 0.001;
        dt_ = 0.02;
        tau_threshold_ = 0.04;
        release_threshold_ = 0.008;
        decay_factor_1_ = 0.9;
        decay_factor_2_ = 0.98;

        delta_theta_.resize(motor_count_, 0.0);
        delta_theta_d_.resize(motor_count_, 0.0);
        tau_ext_f_.resize(motor_count_, 0.0);
        present_position_.resize(motor_count_, 0.0);
        present_current_.resize(motor_count_, 0.0);
        position_ref_.resize(motor_count_, 0.0);

        for (int i = 0; i < motor_count_; ++i)
        {
            std::string state_topic = "/motor_" + std::to_string(i) + "/state";
            std::string goal_topic = "/motor_" + std::to_string(i) + "/goal_position";

            // bind index i to each callback
            ros::Subscriber sub = nh_.subscribe<joint_controller::MotorState>(
                state_topic, 10,
                boost::bind(&AdmittanceController::stateCallback, this, _1, i));
            subs_.push_back(sub);

            ros::Publisher pub = nh_.advertise<std_msgs::Float64>(goal_topic, 10);
            pubs_.push_back(pub);
        }
    }

    void setParam(double M, double B, double K)
    {
        M_ = M;
        B_ = B;
        K_ = K;
    }

    void setThreshold(double tau_threshold, double release_threshold)
    {
        tau_threshold_ = tau_threshold;
        release_threshold_ = release_threshold;
    }

    void stateCallback(const joint_controller::MotorState::ConstPtr &msg, int id)
    {
        present_position_[id] = msg->position;
        present_current_[id] = msg->current;
    }

    void update()
    {
        for (int i = 0; i < motor_count_; ++i)
        {
            double goal_position = position_ref_[i];

            double tau_ext = -present_current_[i] * mA_to_A_ * Kt_;
            tau_ext_f_[i] = alpha_ * tau_ext_f_[i] + (1.0 - alpha_) * tau_ext;
            double tau_abs = std::abs(tau_ext_f_[i]);

            // convert
            double delta_theta_rad = deg2rad(delta_theta_[i]);
            double delta_theta_d_rad = deg2rad(delta_theta_d_[i]);

            if (tau_abs <= release_threshold_)
            {
                delta_theta_d_rad = 0.0;
                delta_theta_rad *= decay_factor_1_;
            }
            else if (tau_abs >= tau_threshold_)
            {
                // Admittance dynamics
                double tau_eff = tau_ext_f_[i] - sign(tau_ext_f_[i]) * tau_threshold_;
                double delta_theta_dd_rad = (tau_eff - B_ * delta_theta_d_rad - K_ * delta_theta_rad) / M_;
                delta_theta_d_rad += dt_ * delta_theta_dd_rad;
                delta_theta_rad += dt_ * delta_theta_d_rad;
            }
            else
            {
                delta_theta_d_rad = 0.0;
                delta_theta_rad *= decay_factor_2_;
            }

            delta_theta_d_[i] = rad2deg(delta_theta_d_rad);
            delta_theta_[i] = rad2deg(delta_theta_rad);

            goal_position += delta_theta_[i];

            std_msgs::Float64 msg;
            msg.data = goal_position;
            pubs_[i].publish(msg);

            ROS_INFO("Motor %d | tau_ext_f: %.4f Nm | goal: %.2f deg | pos: %.2f deg",
                     i + 1, tau_ext_f_[i], goal_position, present_position_[i]);
        }
    }

private:
    ros::NodeHandle nh_;
    int motor_count_;

    std::vector<ros::Subscriber> subs_;
    std::vector<ros::Publisher> pubs_;
    std::vector<double> position_ref_;

    double M_, B_, K_, alpha_, Kt_, mA_to_A_, dt_;
    double tau_threshold_, release_threshold_;
    double decay_factor_1_, decay_factor_2_;
    std::vector<double> delta_theta_, delta_theta_d_, tau_ext_f_;
    std::vector<double> present_position_, present_current_;

    double deg2rad(double deg) const { return deg * M_PI / 180.0; }
    double rad2deg(double rad) const { return rad * 180.0 / M_PI; }
    double sign(double x) const { return (x > 0) - (x < 0); }
};
