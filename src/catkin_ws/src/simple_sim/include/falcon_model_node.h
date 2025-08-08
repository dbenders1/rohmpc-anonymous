#ifndef FALCON_MODEL_NODE_H
#define FALCON_MODEL_NODE_H

#include <cmath>
#include <random>
#include <vector>
#include <Eigen/Dense>

#include <ros/ros.h>

#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <rosgraph_msgs/Clock.h>

#include "simple_sim/DroneFalconStepControl.h"
#include "simple_sim/DroneFalconOutput.h"
#include "simple_sim/GetPhysicsProperties.h"

#define FLOAT_TOL 1e-6

class RandomGaussianGenerator {
public:
    RandomGaussianGenerator(int seed, const Eigen::VectorXd& mean, const Eigen::VectorXd& stddev)
        : generator_(seed), mean_(mean), stddev_(stddev){};
    ~RandomGaussianGenerator(){};
    Eigen::VectorXd generate();

private:
    std::mt19937 generator_;
    std::normal_distribution<double> standard_gaussian_distribution_ = std::normal_distribution<double>(0.0, 1.0);
    Eigen::VectorXd mean_;
    Eigen::VectorXd stddev_;
};

class RandomUniformGenerator {
public:
    RandomUniformGenerator(int seed, const Eigen::VectorXd& mean, const Eigen::VectorXd& stddev)
        : generator_(seed), mean_(mean), stddev_(stddev){};
    ~RandomUniformGenerator(){};
    Eigen::VectorXd generate();
private:
    std::mt19937 generator_;
    std::uniform_real_distribution<double> standard_uniform_distribution_ = std::uniform_real_distribution<double>(-1.0, 1.0);
    Eigen::VectorXd mean_;
    Eigen::VectorXd stddev_;
};

class FalconRK4System
{
public:
    FalconRK4System()
    {
        // Initialize parameters with reasonable values
        g = 9.8124; // m/s^2
        mass = 0.617; // kg
        kappa = 0.022;
        B_allocation << 1, 1, 1, 1,
                        -0.075, 0.075, -0.075, 0.075,
                        -0.075, 0.075, 0.075, -0.075,
                        -kappa, -kappa, kappa, kappa;
        inertia << 0.00164, 0, 0,
                   0, 0.00184, 0,
                   0, 0, 0.0030;
        thrust_map << 1.562522e-6, 0.0, 0.0; // thrust mapping coefficients
        kd << 0, 0, 0,
              0, 0, 0,
              0, 0, 0; // drag coefficients in x, y, and z directions
        E << Eigen::MatrixXd::Identity(12, 12);
    };
    ~FalconRK4System(){};
    Eigen::VectorXd rk4(const Eigen::VectorXd &x, const Eigen::VectorXd &u, const Eigen::VectorXd &w, const double dt);

    Eigen::VectorXd f(const Eigen::VectorXd &x, const Eigen::Vector4d &u, const Eigen::VectorXd &w)
    {
        double phi = x[3];
        double theta = x[4];
        double psi = x[5];
        Eigen::Vector3d v = x.segment<3>(6);
        Eigen::Vector3d wb = x.segment<3>(9);

        double t = (B_allocation.row(0) * u)(0);
        Eigen::Vector3d tau = B_allocation.block<3, 4>(1, 0) * u;

        Eigen::Matrix3d Rc = get_rot_matrix_coordinates(phi, theta, psi);
        Eigen::Matrix3d Rr = get_rot_matrix_rates(phi, theta);

        Eigen::VectorXd dxdt(12);
        dxdt.segment<3>(0) = v;
        dxdt.segment<3>(3) = Rr * wb;
        dxdt.segment<3>(6) = Eigen::Vector3d(0, 0, -g) + Rc * (Eigen::Vector3d(0, 0, t / mass) - kd * Rc.transpose() * v);
        dxdt.segment<3>(9) = inertia.inverse() * (tau - wb.cross(inertia * wb));

        dxdt += E * w;

        return dxdt;
    }

    // Return linear acceleration in body frame
    Eigen::Vector3d imu_lin_acc(const Eigen::VectorXd &x, const Eigen::Vector4d &u)
    {
        double z = x[2];
        double phi = x[3];
        double theta = x[4];
        double psi = x[5];
        Eigen::Vector3d v = x.segment<3>(6);

        double t = (B_allocation.row(0) * u)(0);

        Eigen::Matrix3d Rc = get_rot_matrix_coordinates(phi, theta, psi);

        Eigen::Vector3d lin_acc = Eigen::Vector3d(0, 0, t / mass) - kd * Rc.transpose() * v;

        // Ensure simple ground constraint
        if (z < FLOAT_TOL)
        {
            lin_acc[2] = 0;
        }

        return lin_acc;
    }

    Eigen::Vector4d motor_speeds_to_thrusts(const Eigen::Vector4d &motor_speeds)
    {
        return thrust_map[0] * motor_speeds.array().square().matrix() + thrust_map[1] * motor_speeds + thrust_map[2] * Eigen::Vector4d::Ones();
    }

    Eigen::Vector3d world_to_body(const Eigen::Vector3d &vec, const Eigen::VectorXd &x)
    {
        double phi = x[3];
        double theta = x[4];
        double psi = x[5];

        Eigen::Matrix3d R = get_rot_matrix_coordinates(phi, theta, psi);
        return R.transpose() * vec;
    }

private:

    Eigen::Matrix3d get_rot_matrix_coordinates(double phi, double theta, double psi)
    {
        Eigen::Matrix3d R_psi;
        R_psi << cos(psi), -sin(psi), 0,
                 sin(psi), cos(psi), 0,
                 0, 0, 1;

        Eigen::Matrix3d R_theta;
        R_theta << cos(theta), 0, sin(theta),
                   0, 1, 0,
                   -sin(theta), 0, cos(theta);

        Eigen::Matrix3d R_phi;
        R_phi << 1, 0, 0,
                 0, cos(phi), -sin(phi),
                 0, sin(phi), cos(phi);

        return R_psi * R_theta * R_phi;
    }

    Eigen::Matrix3d get_rot_matrix_rates(double phi, double theta)
    {
        Eigen::Matrix3d Rr;
        Rr << 1, sin(phi) * tan(theta), cos(phi) * tan(theta),
              0, cos(phi), -sin(phi),
              0, sin(phi) / cos(theta), cos(phi) / cos(theta);

        return Rr;
    }

    Eigen::Matrix<double, 4, 4> B_allocation;
    Eigen::Matrix<double, 3, 3> inertia;
    double mass;
    double g;
    // double motor_tau;
    double kappa;
    Eigen::Vector3d thrust_map;
    Eigen::Matrix<double, 3, 3> kd;
    Eigen::Matrix<double, 12, 6> E;
    // Eigen::Matrix<double, 16, 10> E;
};

class DroneFalconModel
{
public:
DroneFalconModel(ros::NodeHandle& nh, Eigen::VectorXd state_init, double model_dt, int n_steps, int seed, bool use_w_random_walk, bool use_w_gaussian_noise, bool use_w_uniform_noise, bool use_eta_gaussian_noise, bool use_eta_uniform_noise, Eigen::VectorXd w_random_walk_correlations, Eigen::VectorXd w_gaussian_mean, Eigen::VectorXd w_gaussian_std, Eigen::VectorXd w_uniform_mean, Eigen::VectorXd w_uniform_std, Eigen::VectorXd w_max, bool add_meas_noise_init, Eigen::VectorXd eta_gaussian_mean, Eigen::VectorXd eta_gaussian_std, Eigen::VectorXd eta_uniform_mean, Eigen::VectorXd eta_uniform_std);
    ~DroneFalconModel();

    void computeDisturbances();
    void computeMeasurementNoise();
    void computeOutput();
    void constructActuatorsMessage();
    void constructImuMessage();
    void constructOdomStateMessage();
    void constructOdomOutputMessage();
    void constructDisturbanceMessage();
    void constructMeasurementNoiseMessage();
    bool getPhysicsProperties(simple_sim::GetPhysicsProperties::Request& req, simple_sim::GetPhysicsProperties::Response& res);
    void integrateState();
    void publish();
    void run();
    void startup();
    void stepControlCallback(const simple_sim::DroneFalconStepControl::ConstPtr& msg);
    void updateTime();
private:
    // ROS nodehandle
    ros::NodeHandle nh_;

    // Model properties
    double model_dt_ = 0.002;
    int n_steps_ = 5;

    // ROS publishers and subscribers
    ros::Subscriber step_control_sub_;
    ros::Publisher clock_publisher_, actuators_pub_, imu_pub_, odom_output_pub_, odom_state_pub_, w_msg_pub_, eta_msg_pub_;

    // ROS servers
    ros::ServiceServer get_physics_properties_server_, step_server_;

    // First run indicator
    bool first_run_ = true;

    // RK4 integration
    FalconRK4System rk4_sys_;
    Eigen::Vector4d u_;
    Eigen::VectorXd w_, x_, eta_, y_;
    tf2::Quaternion q_x_, q_y_;
    bool use_w_random_walk_, use_w_gaussian_noise_, use_w_uniform_noise_, add_meas_noise_init_, use_eta_gaussian_noise_, use_eta_uniform_noise_, use_w_, use_eta_;
    Eigen::VectorXd w_random_walk_correlations_, w_max_;
    RandomGaussianGenerator rg_g_w_, rg_g_eta_;
    RandomUniformGenerator rg_u_w_, rg_u_eta_;

    // Messages
    double time_in_sec_ = 0;
    rosgraph_msgs::Clock curr_time_;
    mav_msgs::Actuators actuators_msg_;
    nav_msgs::Odometry odom_output_msg_, odom_state_msg_;
    sensor_msgs::Imu imu_msg_;
    simple_sim::DroneFalconOutput w_msg_, eta_msg_;

    // Define input, states, and output indices
    struct X {
        enum IDX : int {
            P = 0,
            PX = 0,
            PY = 1,
            PZ = 2,
            NP = 3,
            EUL = 3,
            EULPHI = 3,
            EULTHETA = 4,
            EULPSI = 5,
            NEUL = 3,
            V = 6,
            VX = 6,
            VY = 7,
            VZ = 8,
            NV = 3,
            WB = 9,
            WBX = 9,
            WBY = 10,
            WBZ = 11,
            NWB = 3,
            NX = 12
        };
    };
    struct Y {
        enum IDX : int {
            P = 0,
            PX = 0,
            PY = 1,
            PZ = 2,
            NP = 3,
            Q = 3,
            QW = 3,
            QX = 4,
            QY = 5,
            QZ = 6,
            NQ = 4,
            V = 7,
            VX = 7,
            VY = 8,
            VZ = 9,
            NV = 3,
            WB = 10,
            WBX = 10,
            WBY = 11,
            WBZ = 12,
            NWB = 3,
            NY = 13
        };
    };
    struct W {
        enum IDX : int {
            P = 0,
            PX = 0,
            PY = 1,
            PZ = 2,
            NP = 3,
            EUL = 3,
            EULPHI = 3,
            EULTHETA = 4,
            EULPSI = 5,
            NEUL = 3,
            V = 6,
            VX = 6,
            VY = 7,
            VZ = 8,
            NV = 3,
            WB = 9,
            WBX = 9,
            WBY = 10,
            WBZ = 11,
            NWB = 3,
            NW = 12
        };
    };
    struct U {
        enum IDX : int {
            WM0C = 0,
            WM1C = 1,
            WM2C = 2,
            WM3C = 3,
            NU = 4
        };
    };
    struct ETA {
        enum IDX : int {
            P = 0,
            PX = 0,
            PY = 1,
            PZ = 2,
            NP = 3,
            EUL = 3,
            EULPHI = 3,
            EULTHETA = 4,
            EULPSI = 5,
            NEUL = 3,
            V = 6,
            VX = 6,
            VY = 7,
            VZ = 8,
            NV = 3,
            WB = 9,
            WBX = 9,
            WBY = 10,
            WBZ = 11,
            NWB = 3,
            NETA = 12
        };
    };
};

#endif // FALCON_MODEL_NODE_H
