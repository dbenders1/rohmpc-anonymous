#include <iostream>
#include <sstream>
#include <thread>

#include "instrumentation_timer.h"
#include "falcon_model_node.h"

Eigen::VectorXd parseArrayArgument(const std::string& arg) {
    std::stringstream ss(arg);
    std::string item;
    std::vector<double> values;

    while (std::getline(ss, item, ',')) {
        values.push_back(std::stod(item));
    }

    Eigen::VectorXd vec = Eigen::VectorXd::Zero(values.size());
    for (size_t i = 0; i < values.size(); ++i) {
        vec(i) = values[i];
    }

    return vec;
}

Eigen::VectorXd RandomGaussianGenerator::generate() {
    Eigen::VectorXd result(mean_.size());
    for (int i = 0; i < mean_.size(); ++i) {
        result[i] = mean_[i] + stddev_[i] * standard_gaussian_distribution_(generator_);
    }
    return result;
}

Eigen::VectorXd RandomUniformGenerator::generate() {
    Eigen::VectorXd result(mean_.size());
    for (int i = 0; i < mean_.size(); ++i) {
        result[i] = mean_[i] + stddev_[i] * standard_uniform_distribution_(generator_);
    }
    return result;
}

Eigen::VectorXd FalconRK4System::rk4(const Eigen::VectorXd &x, const Eigen::VectorXd &u, const Eigen::VectorXd &w, const double dt)
{
    Eigen::VectorXd k1 = f(x, u, w);
    Eigen::VectorXd k2 = f(x + dt / 2 * k1, u, w);
    Eigen::VectorXd k3 = f(x + dt / 2 * k2, u, w);
    Eigen::VectorXd k4 = f(x + dt * k3, u, w);
    return x + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
}

DroneFalconModel::DroneFalconModel(ros::NodeHandle& nh, Eigen::VectorXd state_init, double model_dt, int n_steps, int seed, bool use_w_random_walk, bool use_w_gaussian_noise, bool use_w_uniform_noise, bool use_eta_gaussian_noise, bool use_eta_uniform_noise, Eigen::VectorXd w_random_walk_correlations, Eigen::VectorXd w_gaussian_mean, Eigen::VectorXd w_gaussian_std, Eigen::VectorXd w_uniform_mean, Eigen::VectorXd w_uniform_std, Eigen::VectorXd w_max, bool add_meas_noise_init, Eigen::VectorXd eta_gaussian_mean, Eigen::VectorXd eta_gaussian_std, Eigen::VectorXd eta_uniform_mean, Eigen::VectorXd eta_uniform_std)
    : nh_(nh),
      model_dt_(model_dt),
      n_steps_(n_steps),
      use_w_random_walk_(use_w_random_walk),
      use_w_gaussian_noise_(use_w_gaussian_noise),
      use_w_uniform_noise_(use_w_uniform_noise),
      add_meas_noise_init_(add_meas_noise_init),
      use_eta_gaussian_noise_(use_eta_gaussian_noise),
      use_eta_uniform_noise_(use_eta_uniform_noise),
      w_random_walk_correlations_(w_random_walk_correlations),
      w_max_(w_max),
      rg_g_w_(seed,w_gaussian_mean,w_gaussian_std),
      rg_g_eta_(seed,eta_gaussian_mean,eta_gaussian_std),
      rg_u_w_(seed,w_uniform_mean,w_uniform_std),
      rg_u_eta_(seed,eta_uniform_mean,eta_uniform_std)
{
    ROS_WARN("[FMN] Initializing Falcon model node");

    // Initialize profiling
    Helpers::Instrumentor::Get().BeginSession("simple_sim", ros::this_node::getName(), std::string("simple_sim_falcon_profiler") + std::string(".json"));

    // Check if the state_init vector has the correct size
    if (state_init.size() != X::IDX::NX) {
        ROS_ERROR("[FMN] State initialization vector must have size %d, but got size %zu", X::IDX::NX, state_init.size());
        throw std::runtime_error("[FMN] State initialization vector size mismatch");
    }

    // Initialize length of data
    u_ = Eigen::Vector4d::Zero();
    w_ = Eigen::VectorXd::Zero(W::IDX::NW);
    x_ = Eigen::VectorXd::Zero(X::IDX::NX);
    eta_ = Eigen::VectorXd::Zero(ETA::IDX::NETA);
    y_ = Eigen::VectorXd::Zero(Y::IDX::NY);

    // Initialize state vector
    x_ = state_init;
    q_x_.setRPY(x_[X::IDX::EULPHI], x_[X::IDX::EULTHETA], x_[X::IDX::EULPSI]);

    // Determine whether disturbances are used
    if (use_w_random_walk_ || use_w_gaussian_noise_ || use_w_uniform_noise_) {
        if (use_w_random_walk_ && !(use_w_gaussian_noise_ || use_w_uniform_noise_)) {
            ROS_WARN("[FMN] Using random walk correlation without noise is not supported, setting use_w_ to false");
            use_w_ = false;
        } else {
            use_w_ = true;
        }
    } else {
        use_w_ = false;
    }

    // Determine whether measurement noise is used
    if (use_eta_gaussian_noise_ || use_eta_uniform_noise_) {
        use_eta_ = true;
    } else {
        use_eta_ = false;
    }

    // Initialize messages
    actuators_msg_.header.frame_id = "falcon/base_link";
    actuators_msg_.angular_velocities.resize(4);
    imu_msg_.header.frame_id = "falcon/imugt_link";
    odom_output_msg_.header.frame_id = "world";
    odom_state_msg_.header.frame_id = "world";
    if (use_w_) {
        w_msg_.header.frame_id = "world";
        w_msg_.y.resize(W::IDX::NW);
    }
    if (use_eta_) {
        eta_msg_.header.frame_id = "world";
        eta_msg_.y.resize(ETA::IDX::NETA);
    }

    // ROS publishers and subscribers
    step_control_sub_ = nh_.subscribe("/step_control", 1, &DroneFalconModel::stepControlCallback, this);
    clock_publisher_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
    actuators_pub_ = nh_.advertise<mav_msgs::Actuators>("/falcon/motor_speed", 1);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/falcon/ground_truth/imu", 1);
    odom_output_pub_ = nh_.advertise<nav_msgs::Odometry>("/falcon/odometry", 1);
    odom_state_pub_ = nh_.advertise<nav_msgs::Odometry>("/falcon/ground_truth/odometry", 1);
    if (use_w_) w_msg_pub_ = nh_.advertise<simple_sim::DroneFalconOutput>("/w", 1);
    if (use_eta_) eta_msg_pub_ = nh_.advertise<simple_sim::DroneFalconOutput>("/eta", 1);
    get_physics_properties_server_ = nh_.advertiseService("/simplesim/get_physics_properties", &DroneFalconModel::getPhysicsProperties, this);

    ROS_INFO_STREAM("[FMN] Initial state: " << x_.transpose());
    ROS_INFO_STREAM("[FMN] Model dt: " << model_dt);
    ROS_INFO_STREAM("[FMN] Model n_steps: " << n_steps);

    startup();

    ROS_WARN("[FMN] Initialized Falcon model node");
}

DroneFalconModel::~DroneFalconModel()
{
    // Save profiling data
    Helpers::Instrumentor::Get().EndSession();
}

void DroneFalconModel::computeDisturbances()
{
    PROFILE_FUNCTION();

    // Process random walk correlation if desired
    if (use_w_random_walk_) {
        w_ = w_random_walk_correlations_.cwiseProduct(w_);
    } else {
        w_.setZero();
    }

    // Add Gaussian and uniform random sequences if desired
    if (use_w_gaussian_noise_) {
        w_ += rg_g_w_.generate();
    }
    if (use_w_uniform_noise_) {
        w_ += rg_u_w_.generate();
    }

    // Clip the disturbances to the maximum values
    for (int i = 0; i < W::IDX::NW; i++) {
        if (w_[i] > w_max_[i]) {
            w_[i] = w_max_[i];
        } else if (w_[i] < -w_max_[i]) {
            w_[i] = -w_max_[i];
        }
    }
}

void DroneFalconModel::computeMeasurementNoise()
{
    PROFILE_FUNCTION();

    eta_ = Eigen::VectorXd::Zero(ETA::IDX::NETA);
    if (use_eta_gaussian_noise_) {
        eta_ += rg_g_eta_.generate();
    }
    if (use_eta_uniform_noise_) {
        eta_ += rg_u_eta_.generate();
    }
}

void DroneFalconModel::computeOutput()
{
    PROFILE_FUNCTION();

    if (use_eta_) {
        if (!first_run_ || add_meas_noise_init_) computeMeasurementNoise();
    }

    // Compute quaternion from Euler angles
    q_y_.setRPY(x_[X::IDX::EULPHI] + eta_[ETA::IDX::EULPHI], x_[X::IDX::EULTHETA] + eta_[ETA::IDX::EULTHETA], x_[X::IDX::EULPSI] + eta_[ETA::IDX::EULPSI]);

    y_[Y::IDX::PX] = x_[X::IDX::PX] + eta_[ETA::IDX::PX];
    y_[Y::IDX::PY] = x_[X::IDX::PY] + eta_[ETA::IDX::PY];
    y_[Y::IDX::PZ] = x_[X::IDX::PZ] + eta_[ETA::IDX::PZ];
    y_[Y::IDX::QW] = q_y_.w();
    y_[Y::IDX::QX] = q_y_.x();
    y_[Y::IDX::QY] = q_y_.y();
    y_[Y::IDX::QZ] = q_y_.z();
    y_[Y::IDX::VX] = x_[X::IDX::VX] + eta_[ETA::IDX::VX];
    y_[Y::IDX::VY] = x_[X::IDX::VY] + eta_[ETA::IDX::VY];
    y_[Y::IDX::VZ] = x_[X::IDX::VZ] + eta_[ETA::IDX::VZ];
    y_[Y::IDX::WBX] = x_[X::IDX::WBX] + eta_[ETA::IDX::WBX];
    y_[Y::IDX::WBY] = x_[X::IDX::WBY] + eta_[ETA::IDX::WBY];
    y_[Y::IDX::WBZ] = x_[X::IDX::WBZ] + eta_[ETA::IDX::WBZ];
}

void DroneFalconModel::constructActuatorsMessage()
{
    PROFILE_FUNCTION();

    // Construct motor speeds message
    actuators_msg_.angular_velocities[0] = u_[U::IDX::WM0C];
    actuators_msg_.angular_velocities[1] = u_[U::IDX::WM1C];
    actuators_msg_.angular_velocities[2] = u_[U::IDX::WM2C];
    actuators_msg_.angular_velocities[3] = u_[U::IDX::WM3C];
}

void DroneFalconModel::constructImuMessage()
{
    PROFILE_FUNCTION();

    // Compute the linear acceleration in body frame
    Eigen::Vector3d imu_lin_acc = rk4_sys_.imu_lin_acc(x_, u_);

    // Construct IMU message
    imu_msg_.angular_velocity.x = x_[X::IDX::WBX];
    imu_msg_.angular_velocity.y = x_[X::IDX::WBY];
    imu_msg_.angular_velocity.z = x_[X::IDX::WBZ];
    imu_msg_.linear_acceleration.x = imu_lin_acc[0];
    imu_msg_.linear_acceleration.y = imu_lin_acc[1];
    imu_msg_.linear_acceleration.z = imu_lin_acc[2];
}

void DroneFalconModel::constructOdomOutputMessage()
{
    PROFILE_FUNCTION();

    // NOTE: linear velocities are given in inertial frame, which is in contrast to the ROS standard!
    odom_output_msg_.pose.pose.position.x = y_[Y::IDX::PX];
    odom_output_msg_.pose.pose.position.y = y_[Y::IDX::PY];
    odom_output_msg_.pose.pose.position.z = y_[Y::IDX::PZ];
    odom_output_msg_.pose.pose.orientation.w = y_[Y::IDX::QW];
    odom_output_msg_.pose.pose.orientation.x = y_[Y::IDX::QX];
    odom_output_msg_.pose.pose.orientation.y = y_[Y::IDX::QY];
    odom_output_msg_.pose.pose.orientation.z = y_[Y::IDX::QZ];
    odom_output_msg_.twist.twist.linear.x = y_[Y::IDX::VX];
    odom_output_msg_.twist.twist.linear.y = y_[Y::IDX::VY];
    odom_output_msg_.twist.twist.linear.z = y_[Y::IDX::VZ];
    odom_output_msg_.twist.twist.angular.x = y_[Y::IDX::WBX];
    odom_output_msg_.twist.twist.angular.y = y_[Y::IDX::WBY];
    odom_output_msg_.twist.twist.angular.z = y_[Y::IDX::WBZ];
}

void DroneFalconModel::constructOdomStateMessage()
{
    PROFILE_FUNCTION();

    // NOTE: linear velocities are given in inertial frame, which is in contrast to the ROS standard!
    odom_state_msg_.pose.pose.position.x = x_[X::IDX::PX];
    odom_state_msg_.pose.pose.position.y = x_[X::IDX::PY];
    odom_state_msg_.pose.pose.position.z = x_[X::IDX::PZ];
    odom_state_msg_.pose.pose.orientation.w = q_x_.w();
    odom_state_msg_.pose.pose.orientation.x = q_x_.x();
    odom_state_msg_.pose.pose.orientation.y = q_x_.y();
    odom_state_msg_.pose.pose.orientation.z = q_x_.z();
    odom_state_msg_.twist.twist.linear.x = x_[X::IDX::VX];
    odom_state_msg_.twist.twist.linear.y = x_[X::IDX::VY];
    odom_state_msg_.twist.twist.linear.z = x_[X::IDX::VZ];
    odom_state_msg_.twist.twist.angular.x = x_[X::IDX::WBX];
    odom_state_msg_.twist.twist.angular.y = x_[X::IDX::WBY];
    odom_state_msg_.twist.twist.angular.z = x_[X::IDX::WBZ];
}

void DroneFalconModel::constructDisturbanceMessage()
{
    PROFILE_FUNCTION();

    w_msg_.y[W::IDX::PX] = w_[W::IDX::PX];
    w_msg_.y[W::IDX::PY] = w_[W::IDX::PY];
    w_msg_.y[W::IDX::PZ] = w_[W::IDX::PZ];
    w_msg_.y[W::IDX::EULPHI] = w_[W::IDX::EULPHI];
    w_msg_.y[W::IDX::EULTHETA] = w_[W::IDX::EULTHETA];
    w_msg_.y[W::IDX::EULPSI] = w_[W::IDX::EULPSI];
    w_msg_.y[W::IDX::VX] = w_[W::IDX::VX];
    w_msg_.y[W::IDX::VY] = w_[W::IDX::VY];
    w_msg_.y[W::IDX::VZ] = w_[W::IDX::VZ];
    w_msg_.y[W::IDX::WBX] = w_[W::IDX::WBX];
    w_msg_.y[W::IDX::WBY] = w_[W::IDX::WBY];
    w_msg_.y[W::IDX::WBZ] = w_[W::IDX::WBZ];
}

void DroneFalconModel::constructMeasurementNoiseMessage()
{
    PROFILE_FUNCTION();

    eta_msg_.y[ETA::IDX::PX] = eta_[ETA::IDX::PX];
    eta_msg_.y[ETA::IDX::PY] = eta_[ETA::IDX::PY];
    eta_msg_.y[ETA::IDX::PZ] = eta_[ETA::IDX::PZ];
    eta_msg_.y[ETA::IDX::EULPHI] = eta_[ETA::IDX::EULPHI];
    eta_msg_.y[ETA::IDX::EULTHETA] = eta_[ETA::IDX::EULTHETA];
    eta_msg_.y[ETA::IDX::EULPSI] = eta_[ETA::IDX::EULPSI];
    eta_msg_.y[ETA::IDX::VX] = eta_[ETA::IDX::VX];
    eta_msg_.y[ETA::IDX::VY] = eta_[ETA::IDX::VY];
    eta_msg_.y[ETA::IDX::VZ] = eta_[ETA::IDX::VZ];
    eta_msg_.y[ETA::IDX::WBX] = eta_[ETA::IDX::WBX];
    eta_msg_.y[ETA::IDX::WBY] = eta_[ETA::IDX::WBY];
    eta_msg_.y[ETA::IDX::WBZ] = eta_[ETA::IDX::WBZ];
}

bool DroneFalconModel::getPhysicsProperties(simple_sim::GetPhysicsProperties::Request& req, simple_sim::GetPhysicsProperties::Response& res)
{
    PROFILE_FUNCTION();

    res.time_step = model_dt_;
    return true;
}

void DroneFalconModel::integrateState()
{
    PROFILE_FUNCTION();

    if (use_w_) computeDisturbances();

    // Integrate state without disturbances
    x_ = rk4_sys_.rk4(x_, u_, w_, model_dt_);
    if (x_[X::IDX::PZ] < 0)
    {
        x_.segment<10>(X::IDX::PZ) = Eigen::VectorXd::Zero(10);
    }

    // Compute quaternion from Euler angles
    q_x_.setRPY(x_[X::IDX::EULPHI], x_[X::IDX::EULTHETA], x_[X::IDX::EULPSI]);

    updateTime();
}

void DroneFalconModel::publish()
{
    PROFILE_FUNCTION();

    // Publish updated time
    curr_time_.clock.fromSec(time_in_sec_);
    clock_publisher_.publish(curr_time_);

    // Publish actuators message
    actuators_msg_.header.stamp.fromSec(time_in_sec_);
    actuators_pub_.publish(actuators_msg_);
    
    // Publish IMU message
    imu_msg_.header.stamp.fromSec(time_in_sec_);
    imu_pub_.publish(imu_msg_);

    // Publish odom output message
    odom_output_msg_.header.stamp.fromSec(time_in_sec_);
    odom_output_pub_.publish(odom_output_msg_);

    // Publish odom state message
    odom_state_msg_.header.stamp.fromSec(time_in_sec_);
    odom_state_pub_.publish(odom_state_msg_);

    // Publish disturbances message if used
    if (use_w_) {
        w_msg_.header.stamp.fromSec(time_in_sec_ - model_dt_); // w_msg_ gets timestamp one in the past since the value was applied to evolve the previous state to the current state
        w_msg_pub_.publish(w_msg_);
    }

    // Publish measurement noise message if used
    if (use_eta_) {
        eta_msg_.header.stamp.fromSec(time_in_sec_);
        eta_msg_pub_.publish(eta_msg_);
    }
}

void DroneFalconModel::run()
{
    PROFILE_FUNCTION();

    if (!first_run_) 
    {
        for (int step_idx = 0; step_idx < n_steps_; step_idx++)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            integrateState();
            computeOutput();
            constructActuatorsMessage();
            constructImuMessage();
            constructOdomOutputMessage();
            constructOdomStateMessage();
            if (use_w_) constructDisturbanceMessage();
            if (use_eta_) constructMeasurementNoiseMessage();
            publish();
        }
    }
    else
    {
        for (int step_idx = 0; step_idx < n_steps_; step_idx++)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            updateTime();
            computeOutput();
            constructActuatorsMessage();
            constructImuMessage();
            constructOdomOutputMessage();
            constructOdomStateMessage();
            if (use_w_) constructDisturbanceMessage();
            if (use_eta_) constructMeasurementNoiseMessage();
            publish();
        }
        first_run_ = false;
    }
}

void DroneFalconModel::startup()
{
    PROFILE_FUNCTION();
}

void DroneFalconModel::stepControlCallback(const simple_sim::DroneFalconStepControl::ConstPtr& msg)
{
    PROFILE_FUNCTION();

    // If steps > 0 run simulation with the given number of steps and the given control command
    if (msg->steps > 0)
    {
        n_steps_ = msg->steps;
        for (int i = 0; i < 4; ++i)
        {
            u_[i] = msg->angular_velocities[i];
        }
        u_ = rk4_sys_.motor_speeds_to_thrusts(u_);
        run();
    } else {
        ROS_ERROR("[FMN] Number of steps must be greater than 0");
    }
}

void DroneFalconModel::updateTime()
{
    PROFILE_FUNCTION();

    time_in_sec_ += model_dt_;
}

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "falcon_model_sim_node");
    ros::NodeHandle nh("~");

    // Read program arguments
    Eigen::VectorXd state_init = parseArrayArgument(std::string(argv[1]));
    double model_dt = 0.002;
    sscanf(argv[2], "%lf", &model_dt);
    int n_steps = 5;
    sscanf(argv[3], "%d", &n_steps);
    int seed;
    sscanf(argv[4], "%d", &seed);
    bool use_w_random_walk, use_w_gaussian_noise, use_w_uniform_noise, add_meas_noise_init, use_eta_gaussian_noise, use_eta_uniform_noise;
    std::string use_w_random_walk_str = argv[5];
    if (use_w_random_walk_str == "true") use_w_random_walk = true;
    else use_w_random_walk = false;
    std::string use_w_gaussian_noise_str = argv[6];
    if (use_w_gaussian_noise_str == "true") use_w_gaussian_noise = true;
    else use_w_gaussian_noise = false;
    std::string use_w_uniform_noise_str = argv[7];
    if (use_w_uniform_noise_str == "true") use_w_uniform_noise = true;
    else use_w_uniform_noise = false;
    std::string add_meas_noise_init_str = argv[8];
    if (add_meas_noise_init_str == "true") add_meas_noise_init = true;
    else add_meas_noise_init = false;
    std::string use_eta_gaussian_noise_str = argv[9];
    if (use_eta_gaussian_noise_str == "true") use_eta_gaussian_noise = true;
    else use_eta_gaussian_noise = false;
    std::string use_eta_uniform_noise_str = argv[10];
    if (use_eta_uniform_noise_str == "true") use_eta_uniform_noise = true;
    else use_eta_uniform_noise = false;
    Eigen::VectorXd w_random_walk_correlations = parseArrayArgument(std::string(argv[11]));
    Eigen::VectorXd w_gaussian_mean = parseArrayArgument(std::string(argv[12]));
    Eigen::VectorXd w_gaussian_std = parseArrayArgument(std::string(argv[13]));
    Eigen::VectorXd w_uniform_mean = parseArrayArgument(std::string(argv[14]));
    Eigen::VectorXd w_uniform_std = parseArrayArgument(std::string(argv[15]));
    Eigen::VectorXd w_max = parseArrayArgument(std::string(argv[16]));
    Eigen::VectorXd eta_gaussian_mean = parseArrayArgument(std::string(argv[17]));
    Eigen::VectorXd eta_gaussian_std = parseArrayArgument(std::string(argv[18]));
    Eigen::VectorXd eta_uniform_mean = parseArrayArgument(std::string(argv[19]));
    Eigen::VectorXd eta_uniform_std = parseArrayArgument(std::string(argv[20]));

    // Create model object
    DroneFalconModel falcon_model(nh, state_init, model_dt, n_steps, seed, use_w_random_walk, use_w_gaussian_noise, use_w_uniform_noise, use_eta_gaussian_noise, use_eta_uniform_noise, w_random_walk_correlations, w_gaussian_mean, w_gaussian_std, w_uniform_mean, w_uniform_std, w_max, add_meas_noise_init, eta_gaussian_mean, eta_gaussian_std, eta_uniform_mean, eta_uniform_std);

    ros::spin();

    return 0;
}
