#include <cstdio>

#include "instrumentation_timer.h"
#include "hovergames_model_node.h"

DroneHovergamesModel::DroneHovergamesModel(ros::NodeHandle& nh, std::vector<double> pos_init, bool run_event_based, double model_dt, int n_steps, double model_rate, bool add_timing_variance)
    : nh_(nh),
      pos_init_(pos_init),
      run_event_based_(run_event_based),
      model_dt_(model_dt),
      n_steps_(n_steps),
      model_rate_(model_rate),
      add_timing_variance_(add_timing_variance),
      timer_(nh.createTimer(ros::Duration(1.0 / model_rate), std::bind(&DroneHovergamesModel::publishStateTimer, this, std::placeholders::_1)))
{
    ROS_WARN("[HMN] Initializing hovergames model node");

    // Stop timer, control starting of timer in startup
    timer_.stop();

    // Initialize random number generator
    if (!run_event_based_ && add_timing_variance_)
    {
        normal_dist_.param(std::normal_distribution<double>::param_type(mean_, stddev_));
    }

    // Initialize profiling
    Helpers::Instrumentor::Get().BeginSession("simple_sim", ros::this_node::getName(), std::string("simple_sim_hovergames_profiler") + std::string(".json"));

    // Data initialization
    rk4_sys_.u.resize(4);
    rk4_sys_.u[0] = 0.0;
    rk4_sys_.u[1] = 0.0;
    rk4_sys_.u[2] = 0.0;
    rk4_sys_.u[3] = 9.81;

    x_cur_.resize(10);
    x_cur_[0] = pos_init_[0];
    x_cur_[1] = pos_init_[1];
    x_cur_[2] = pos_init_[2];
    x_cur_[3] = 0.0;
    x_cur_[4] = 0.0;
    x_cur_[5] = 0.0;
    x_cur_[6] = 0.0;
    x_cur_[7] = 0.0;
    x_cur_[8] = 0.0;
    x_cur_[9] = 9.81;

    state_msg_.header.stamp = ros::Time::now();
    state_msg_.header.frame_id = "map";
    state_msg_.child_frame_id = "base_link";
    state_msg_.pose.pose.position.x = pos_init_[0];
    state_msg_.pose.pose.position.y = pos_init_[1];
    state_msg_.pose.pose.position.z = pos_init_[2];
    state_msg_.pose.pose.orientation.x = 0.0;
    state_msg_.pose.pose.orientation.y = 0.0;
    state_msg_.pose.pose.orientation.z = 0.0;
    state_msg_.pose.pose.orientation.w = 1.0;
    state_msg_.twist.twist.linear.x = 0.0;
    state_msg_.twist.twist.linear.y = 0.0;
    state_msg_.twist.twist.linear.z = 0.0;
    state_msg_.twist.twist.angular.x = 0.0;
    state_msg_.twist.twist.angular.y = 0.0;
    state_msg_.twist.twist.angular.z = 0.0;

    transform_msg_.header.stamp = state_msg_.header.stamp;
    transform_msg_.header.frame_id = state_msg_.header.frame_id;
    transform_msg_.child_frame_id = state_msg_.child_frame_id;
    transform_msg_.transform.translation.x = state_msg_.pose.pose.position.x;
    transform_msg_.transform.translation.y = state_msg_.pose.pose.position.y;
    transform_msg_.transform.translation.z = state_msg_.pose.pose.position.z;
    transform_msg_.transform.rotation.x = state_msg_.pose.pose.orientation.x;
    transform_msg_.transform.rotation.y = state_msg_.pose.pose.orientation.y;
    transform_msg_.transform.rotation.z = state_msg_.pose.pose.orientation.z;
    transform_msg_.transform.rotation.w = state_msg_.pose.pose.orientation.w;

    // ROS publishers and subscribers
    control_sub_ = nh_.subscribe("/drone_hovergames/control", 1, &DroneHovergamesModel::controlCallback, this);
    state_pub_ = nh_.advertise<nav_msgs::Odometry>("/drone_hovergames/state", 1);

    ROS_INFO_STREAM("[HMN] Initial positon: [" << pos_init_[0] << ", " << pos_init_[1] << ", " << pos_init_[2] << "]");
    ROS_INFO_STREAM("[HMN] Run event based: " << run_event_based_);
    ROS_INFO_STREAM("[HMN] Model dt: " << model_dt);
    ROS_INFO_STREAM("[HMN] Model n_steps: " << n_steps);
    ROS_INFO_STREAM("[HMN] Model rate: " << model_rate_);
    ROS_INFO_STREAM("[HMN] Add timing variance: " << add_timing_variance_);

    t_final_ = n_steps_ * model_dt_;

    startup();

    ROS_WARN("[HMN] Initialized hovergames model node");
}

DroneHovergamesModel::~DroneHovergamesModel()
{
    // Save profiling data
    Helpers::Instrumentor::Get().EndSession();
}

void DroneHovergamesModel::startup()
{
    PROFILE_FUNCTION();
    // ROS_INFO("startup");

    // Always sleep to let occupancy grid map be published first
    ros::Duration(1.1).sleep();

    // Depending on synchronized mode or not: first publish state or start timer
    if (run_event_based_)
    {
        publishState();
    }
    else
    {
        timer_.start();
    }
}

void DroneHovergamesModel::controlCallback(const simple_sim::DroneHovergamesControl::ConstPtr& msg)
{
    PROFILE_FUNCTION();
    // ROS_INFO("controlCallback");

    // Update current control input
    rk4_sys_.u.insert(rk4_sys_.u.begin(), msg->u.begin(), msg->u.end());

    integrate();

    if (run_event_based_)
    {
        ros::Duration(0.002).sleep();
        publishState();
    }
}

void DroneHovergamesModel::integrate()
{
    PROFILE_FUNCTION();

    // Update state using RK4
    rk4_integrator_.integrate_const(rk4_sys_, x_cur_, t_cur_, t_final_, model_dt_);

    // Save resulting state
    for (int i = 0; i < 10; i++) x_cur_[i] = rk4_integrator_.observer_.x[n_steps_][i];

    // Convert ZYX Euler angles to quaternion
    // NOTE: ZYX Euler angles give the same result as XYZ fixed angles, which is what setRPY uses
    q_.setRPY(x_cur_[6], x_cur_[7], x_cur_[8]);

    // Fill odometry message
    state_msg_.pose.pose.position.x = x_cur_[0];
    state_msg_.pose.pose.position.y = x_cur_[1];
    state_msg_.pose.pose.position.z = x_cur_[2];
    state_msg_.twist.twist.linear.x = x_cur_[3];
    state_msg_.twist.twist.linear.y = x_cur_[4];
    state_msg_.twist.twist.linear.z = x_cur_[5];
    state_msg_.pose.pose.orientation.x = q_.x();
    state_msg_.pose.pose.orientation.y = q_.y();
    state_msg_.pose.pose.orientation.z = q_.z();
    state_msg_.pose.pose.orientation.w = q_.w();

    // Fill transform message
    transform_msg_.transform.translation.x = state_msg_.pose.pose.position.x;
    transform_msg_.transform.translation.y = state_msg_.pose.pose.position.y;
    transform_msg_.transform.translation.z = state_msg_.pose.pose.position.z;
    transform_msg_.transform.rotation.x = state_msg_.pose.pose.orientation.x;
    transform_msg_.transform.rotation.y = state_msg_.pose.pose.orientation.y;
    transform_msg_.transform.rotation.z = state_msg_.pose.pose.orientation.z;
    transform_msg_.transform.rotation.w = state_msg_.pose.pose.orientation.w;
}

void DroneHovergamesModel::publishState()
{
    PROFILE_FUNCTION();
    // ROS_INFO("publishState");

    // Publish updated state
    state_msg_.header.stamp = ros::Time::now();
    state_pub_.publish(state_msg_);

    // Publish transform together with state
    publishTransform();
}

void DroneHovergamesModel::publishStateTimer(const ros::TimerEvent& event)
{
    PROFILE_FUNCTION();
    // ROS_INFO("publishStateTimer");

    // TODO: timer seems to be doubly-scheduled with add_timing_variance_ = true
    if (add_timing_variance_)
    {
        double random_dt = normal_dist_(random_generator_);
        double dur = 1.0 / model_rate_ + random_dt;
        ros::Duration duration = ros::Duration(dur);
        timer_.setPeriod(duration, false);
    }

    publishState();
}

void DroneHovergamesModel::publishTransform()
{
    PROFILE_FUNCTION();
    // ROS_INFO("publishTransform");

    // Publish updated transform with the same timestamp as the state
    transform_msg_.header.stamp = state_msg_.header.stamp;
    tf_broadcaster_.sendTransform(transform_msg_);
}

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "hovergames_model_sim_node");
    ros::NodeHandle nh("~");

    // Read program arguments
    double x_init, y_init, z_init;
    std::vector<double> pos_init = {0, 0, 1};
    bool run_event_based = true;
    double model_dt = 0.05;
    int n_steps = 1;
    double model_rate = 20;
    bool add_timing_variance = false;
    sscanf(argv[1], "%lf", &x_init);
    sscanf(argv[2], "%lf", &y_init);
    sscanf(argv[3], "%lf", &z_init);
    pos_init[0] = x_init;
    pos_init[1] = y_init;
    pos_init[2] = z_init;
    std::string run_event_based_str(argv[4]);
    if (run_event_based_str == "true")
    {
        run_event_based = true;
    }
    else if (run_event_based_str == "false")
    {
        run_event_based = false;
    }
    else
    {
        ROS_ERROR_STREAM("Invalid argument for run_event_based: " << run_event_based_str.c_str());
        return -1;
    }
    sscanf(argv[5], "%lf", &model_dt);
    sscanf(argv[6], "%d", &n_steps);
    sscanf(argv[7], "%lf", &model_rate);
    std::string add_timing_variance_str(argv[8]);
    if (add_timing_variance_str == "true")
    {
        add_timing_variance = true;
    }
    else if (add_timing_variance_str == "false")
    {
        add_timing_variance = false;
    }
    else
    {
        ROS_ERROR_STREAM("Invalid argument for add_timing_variance: " << add_timing_variance_str.c_str());
        return -1;
    }

    // Let other nodes start up
    ros::Rate loop_rate(0.5);
    loop_rate.sleep();

    // Create model object
    DroneHovergamesModel hovergames_model(nh, pos_init, run_event_based, model_dt, n_steps, model_rate, add_timing_variance);

    ros::spin();

    return 0;
}
