#ifndef HOVERGAMES_MODEL_NODE_H
#define HOVERGAMES_MODEL_NODE_H

#include <cmath>
#include <random>
#include <vector>

#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include "rk4.h"
#include "ros_timer_handler.h"

#include "simple_sim/DroneHovergamesControl.h"

typedef std::vector<double> rk4_state_type_;
template<class StateType>
class HovergamesRK4System : public RK4::System<StateType>
{
public:
    HovergamesRK4System()
    {
        u.resize(4);
        u[0] = 0.0;
        u[1] = 0.0;
        u[2] = 0.0;
        u[3] = 0.0;
    };
    ~HovergamesRK4System(){};

    std::vector<double> u;

    // x = [x;y;z;vx;vy;vz;phi;theta;psi]
    void operator()(const StateType &x, StateType &dxdt, double t)
    {
        const double A = -5.55;
        const double B = 5.55;

        const double A_yaw = -1.773;
        const double B_yaw = 1.773;

        const double A_thrust = -20;
        const double B_thrust = 20;
        
        // NOTE: these equations should match the equations in the corresponding model in python_forces_code/dynamics.py!
        dxdt[0] = x[3];
        dxdt[1] = x[4];
        dxdt[2] = x[5];
        dxdt[3] = x[9]*(sin(x[6])*sin(x[8]) + cos(x[6])*sin(x[7])*cos(x[8]));
        dxdt[4] = x[9]*(-sin(x[6])*cos(x[8]) + cos(x[6])*sin(x[7])*sin(x[8]));
        dxdt[5] = x[9]*cos(x[6])*cos(x[7]) - g;
        dxdt[6] = A * x[6] + B * u[0];
        dxdt[7] = A * x[7] + B * u[1];
        dxdt[8] = A_yaw * x[8]+ B_yaw * u[2];
        dxdt[9] = A_thrust * x[9] + B_thrust * u[3];
    };

private:
    const double g = 9.81;
};

class DroneHovergamesModel
{
public:
    DroneHovergamesModel(ros::NodeHandle& nh, std::vector<double> pos_init, bool run_event_based, double model_dt, int n_steps, double model_rate, bool add_timing_variance);
    ~DroneHovergamesModel();

    void startup();
    void controlCallback(const simple_sim::DroneHovergamesControl::ConstPtr& msg);
    void integrate();
    void publishState();
    void publishStateTimer(const ros::TimerEvent& event);
    void publishTransform();
private:
    // ROS nodehandle
    ros::NodeHandle nh_;

    // Model properties
    std::vector<double> pos_init_ = {0, 0, 1};
    bool run_event_based_ = true;
    double model_dt_ = 0.05;
    int n_steps_ = 1;
    double model_rate_ = 100;
    bool add_timing_variance_ = false;

    // Timing
    Helpers::RosTimerHandler timer_;
    std::mt19937 random_generator_{};
    double mean_ = 0.0, stddev_ = 0.0001;
    std::normal_distribution<double> normal_dist_;

    // ROS publishers and subscribers
    ros::Subscriber control_sub_;
    ros::Publisher state_pub_;
    
    // ROS transform publisher
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // RK4 integration
    const double t_cur_ = 0.0;
    double t_final_ = n_steps_ * model_dt_;
    rk4_state_type_ x_cur_;
    HovergamesRK4System<rk4_state_type_> rk4_sys_;
    RK4::Integrator<rk4_state_type_, HovergamesRK4System<rk4_state_type_>> rk4_integrator_;
    tf2::Quaternion q_;

    // Messages
    nav_msgs::Odometry state_msg_;
    geometry_msgs::TransformStamped transform_msg_;
};

#endif // HOVERGAMES_MODEL_NODE_H
