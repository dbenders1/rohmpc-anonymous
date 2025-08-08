/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// MODULE
#include "rotors_gazebo_plugins/gazebo_odometry_plugin.h"

// SYSTEM
#include <chrono>
#include <cmath>
#include <iostream>

// 3RD PARTY
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// USER
#include <rotors_gazebo_plugins/common.h>
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"
#include "PoseStamped.pb.h"
#include "PoseWithCovarianceStamped.pb.h"
#include "TransformStamped.pb.h"
#include "TransformStampedWithFrameIds.pb.h"
#include "Vector3dStamped.pb.h"

Eigen::VectorXd RandomUniformGenerator::generate() {
    Eigen::VectorXd result(mean_.size());
    for (int i = 0; i < mean_.size(); ++i) {
        result[i] = mean_[i] + stddev_[i] * standard_uniform_distribution_(generator_);
    }
    return result;
}

namespace gazebo {

GazeboOdometryPlugin::~GazeboOdometryPlugin() {
}

void GazeboOdometryPlugin::Load(physics::ModelPtr _model,
                                sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  SdfVector3 noise_normal_position;
  SdfVector3 noise_normal_quaternion;
  SdfVector3 noise_normal_linear_velocity;
  SdfVector3 noise_normal_angular_velocity;
  SdfVector3 noise_uniform_position;
  SdfVector3 noise_uniform_quaternion;
  SdfVector3 noise_uniform_linear_velocity;
  SdfVector3 noise_uniform_angular_velocity;
  const SdfVector3 zeros3(0.0, 0.0, 0.0);

  odometry_queue_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_odometry_plugin] Please specify a robotNamespace.\n";

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_odometry_plugin] Please specify a linkName.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_odometry_plugin] Couldn't find specified link \""
            << link_name_ << "\".");

  if (_sdf->HasElement("covarianceImage")) {
    std::string image_name =
        _sdf->GetElement("covarianceImage")->Get<std::string>();
    covariance_image_ = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
    if (covariance_image_.data == NULL)
      gzerr << "loading covariance image " << image_name << " failed"
            << std::endl;
    else
      gzlog << "loading covariance image " << image_name << " successful"
            << std::endl;
  }

  if (_sdf->HasElement("randomEngineSeed")) {
    seed_ = _sdf->GetElement("randomEngineSeed")->Get<unsigned int>();
  } else {
    seed_ = std::chrono::system_clock::now().time_since_epoch().count();
  }
  random_generator_.seed(seed_);

  getSdfParam<std::string>(_sdf, "poseTopic", pose_pub_topic_, pose_pub_topic_);
  getSdfParam<std::string>(_sdf, "poseWithCovarianceTopic",
                           pose_with_covariance_stamped_pub_topic_,
                           pose_with_covariance_stamped_pub_topic_);
  getSdfParam<std::string>(_sdf, "positionTopic", position_stamped_pub_topic_,
                           position_stamped_pub_topic_);
  getSdfParam<std::string>(_sdf, "transformTopic", transform_stamped_pub_topic_,
                           transform_stamped_pub_topic_);
  getSdfParam<std::string>(_sdf, "odometryTopic", odometry_pub_topic_,
                           odometry_pub_topic_);
  getSdfParam<std::string>(_sdf, "parentFrameId", parent_frame_id_,
                           parent_frame_id_);
  getSdfParam<std::string>(_sdf, "childFrameId", child_frame_id_,
                           child_frame_id_);
  getSdfParam<SdfVector3>(_sdf, "noiseNormalPosition", noise_normal_position,
                          zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseNormalQuaternion",
                          noise_normal_quaternion, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseNormalLinearVelocity",
                          noise_normal_linear_velocity, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseNormalAngularVelocity",
                          noise_normal_angular_velocity, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformPosition", noise_uniform_position,
                          zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformQuaternion",
                          noise_uniform_quaternion, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformLinearVelocity",
                          noise_uniform_linear_velocity, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformAngularVelocity",
                          noise_uniform_angular_velocity, zeros3);
  getSdfParam<int>(_sdf, "measurementDelay", measurement_delay_,
                   measurement_delay_);
  getSdfParam<int>(_sdf, "measurementDivisor", measurement_divisor_,
                   measurement_divisor_);
  getSdfParam<double>(_sdf, "unknownDelay", unknown_delay_, unknown_delay_);
  getSdfParam<double>(_sdf, "covarianceImageScale", covariance_image_scale_,
                      covariance_image_scale_);
  getSdfParam<bool>(_sdf, "addNoiseInit", add_noise_init_, false);
  getSdfParam<bool>(_sdf, "addNoise", add_noise_, false);

  random_u_generator_.setSeed(seed_);
  random_u_generator_.setMean(Eigen::VectorXd::Zero(12));
  Eigen::VectorXd stddev(12);
  stddev << noise_uniform_position.X(), noise_uniform_position.Y(),
      noise_uniform_position.Z(), noise_uniform_quaternion.X(),
      noise_uniform_quaternion.Y(), noise_uniform_quaternion.Z(),
      noise_uniform_linear_velocity.X(), noise_uniform_linear_velocity.Y(),
      noise_uniform_linear_velocity.Z(), noise_uniform_angular_velocity.X(),
      noise_uniform_angular_velocity.Y(), noise_uniform_angular_velocity.Z();
  random_u_generator_.setStddev(stddev);

  parent_link_ = world_->EntityByName(parent_frame_id_);
  if (parent_link_ == NULL && parent_frame_id_ != kDefaultParentFrameId) {
    gzthrow("[gazebo_odometry_plugin] Couldn't find specified parent link \""
            << parent_frame_id_ << "\".");
  }

  position_n_[0] = NormalDistribution(0, noise_normal_position.X());
  position_n_[1] = NormalDistribution(0, noise_normal_position.Y());
  position_n_[2] = NormalDistribution(0, noise_normal_position.Z());

  attitude_n_[0] = NormalDistribution(0, noise_normal_quaternion.X());
  attitude_n_[1] = NormalDistribution(0, noise_normal_quaternion.Y());
  attitude_n_[2] = NormalDistribution(0, noise_normal_quaternion.Z());

  linear_velocity_n_[0] =
      NormalDistribution(0, noise_normal_linear_velocity.X());
  linear_velocity_n_[1] =
      NormalDistribution(0, noise_normal_linear_velocity.Y());
  linear_velocity_n_[2] =
      NormalDistribution(0, noise_normal_linear_velocity.Z());

  angular_velocity_n_[0] =
      NormalDistribution(0, noise_normal_angular_velocity.X());
  angular_velocity_n_[1] =
      NormalDistribution(0, noise_normal_angular_velocity.Y());
  angular_velocity_n_[2] =
      NormalDistribution(0, noise_normal_angular_velocity.Z());

  position_u_[0] = UniformDistribution(-noise_uniform_position.X(),
                                       noise_uniform_position.X());
  position_u_[1] = UniformDistribution(-noise_uniform_position.Y(),
                                       noise_uniform_position.Y());
  position_u_[2] = UniformDistribution(-noise_uniform_position.Z(),
                                       noise_uniform_position.Z());

  attitude_u_[0] = UniformDistribution(-noise_uniform_quaternion.X(),
                                       noise_uniform_quaternion.X());
  attitude_u_[1] = UniformDistribution(-noise_uniform_quaternion.Y(),
                                       noise_uniform_quaternion.Y());
  attitude_u_[2] = UniformDistribution(-noise_uniform_quaternion.Z(),
                                       noise_uniform_quaternion.Z());

  linear_velocity_u_[0] = UniformDistribution(
      -noise_uniform_linear_velocity.X(), noise_uniform_linear_velocity.X());
  linear_velocity_u_[1] = UniformDistribution(
      -noise_uniform_linear_velocity.Y(), noise_uniform_linear_velocity.Y());
  linear_velocity_u_[2] = UniformDistribution(
      -noise_uniform_linear_velocity.Z(), noise_uniform_linear_velocity.Z());

  angular_velocity_u_[0] = UniformDistribution(
      -noise_uniform_angular_velocity.X(), noise_uniform_angular_velocity.X());
  angular_velocity_u_[1] = UniformDistribution(
      -noise_uniform_angular_velocity.Y(), noise_uniform_angular_velocity.Y());
  angular_velocity_u_[2] = UniformDistribution(
      -noise_uniform_angular_velocity.Z(), noise_uniform_angular_velocity.Z());

  // Fill in covariance. We omit uniform noise here.
  Eigen::Map<Eigen::Matrix<double, 6, 6> > pose_covariance(
      pose_covariance_matrix_.data());
  Eigen::Matrix<double, 6, 1> pose_covd;

  pose_covd << noise_normal_position.X() * noise_normal_position.X(),
      noise_normal_position.Y() * noise_normal_position.Y(),
      noise_normal_position.Z() * noise_normal_position.Z(),
      noise_normal_quaternion.X() * noise_normal_quaternion.X(),
      noise_normal_quaternion.Y() * noise_normal_quaternion.Y(),
      noise_normal_quaternion.Z() * noise_normal_quaternion.Z();
  pose_covariance = pose_covd.asDiagonal();

  // Fill in covariance. We omit uniform noise here.
  Eigen::Map<Eigen::Matrix<double, 6, 6> > twist_covariance(
      twist_covariance_matrix_.data());
  Eigen::Matrix<double, 6, 1> twist_covd;

  twist_covd << noise_normal_linear_velocity.X() *
                    noise_normal_linear_velocity.X(),
      noise_normal_linear_velocity.Y() * noise_normal_linear_velocity.Y(),
      noise_normal_linear_velocity.Z() * noise_normal_linear_velocity.Z(),
      noise_normal_angular_velocity.X() * noise_normal_angular_velocity.X(),
      noise_normal_angular_velocity.Y() * noise_normal_angular_velocity.Y(),
      noise_normal_angular_velocity.Z() * noise_normal_angular_velocity.Z();
  twist_covariance = twist_covd.asDiagonal();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboOdometryPlugin::OnUpdate, this, _1));
  worldUpdateEndConnection_ = event::Events::ConnectWorldUpdateEnd(
    boost::bind(&GazeboOdometryPlugin::OnWorldUpdateEnd, this));

  // Create ROS node here to speed up publishing of ROS message
  ros_node_handle_ = new ros::NodeHandle();
  ros_odometry_output_pub_ = ros_node_handle_->advertise<nav_msgs::Odometry>(
      "/falcon/odometry", 1);
  ros_odometry_state_pub_ = ros_node_handle_->advertise<nav_msgs::Odometry>(
      "/falcon/ground_truth/odometry", 1);
  meas_noise_pub_ = ros_node_handle_->advertise<rotors_comm::DroneFalconOutput>("/eta", 1);
}

// This gets called by the world update start event.
void GazeboOdometryPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }
}

void GazeboOdometryPlugin::OnWorldUpdateEnd() {
  // This is called after the physics update.
  // We can use this to publish the odometry message.
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // C denotes child frame, P parent frame, and W world frame.
  // Further C_pose_W_P denotes pose of P wrt. W expressed in C.
  ignition::math::Pose3d W_pose_W_C = link_->WorldCoGPose();
  ignition::math::Vector3d C_linear_velocity_W_C = link_->RelativeLinearVel();
  ignition::math::Vector3d W_linear_velocity_W_C = link_->WorldCoGLinearVel();
  ignition::math::Vector3d C_angular_velocity_W_C = link_->RelativeAngularVel();

  ignition::math::Vector3d gazebo_linear_velocity = W_linear_velocity_W_C;
  ignition::math::Vector3d gazebo_angular_velocity = C_angular_velocity_W_C;
  ignition::math::Pose3d gazebo_pose = W_pose_W_C;

  if (parent_frame_id_ != kDefaultParentFrameId) {
    ignition::math::Pose3d W_pose_W_P = parent_link_->WorldPose();
    ignition::math::Vector3d P_linear_velocity_W_P = parent_link_->RelativeLinearVel();
    ignition::math::Vector3d P_angular_velocity_W_P =
        parent_link_->RelativeAngularVel();
    ignition::math::Pose3d C_pose_P_C_ = W_pose_W_C - W_pose_W_P;
    ignition::math::Vector3d C_linear_velocity_P_C;
    // \prescript{}{C}{\dot{r}}_{PC} = -R_{CP}
    //       \cdot \prescript{}{P}{\omega}_{WP} \cross \prescript{}{P}{r}_{PC}
    //       + \prescript{}{C}{v}_{WC}
    //                                 - R_{CP} \cdot \prescript{}{P}{v}_{WP}
    C_linear_velocity_P_C =
        -C_pose_P_C_.Rot().Inverse() *
            P_angular_velocity_W_P.Cross(C_pose_P_C_.Pos()) +
        C_linear_velocity_W_C -
        C_pose_P_C_.Rot().Inverse() * P_linear_velocity_W_P;

    // \prescript{}{C}{\omega}_{PC} = \prescript{}{C}{\omega}_{WC}
    //       - R_{CP} \cdot \prescript{}{P}{\omega}_{WP}
    gazebo_angular_velocity =
        C_angular_velocity_W_C -
        C_pose_P_C_.Rot().Inverse() * P_angular_velocity_W_P;
    gazebo_linear_velocity = C_linear_velocity_P_C;
    gazebo_pose = C_pose_P_C_;
  }

  // Compute measurement noise
  if ((!first_run_ || add_noise_init_) && add_noise_) {
    meas_noise_ = random_u_generator_.generate();
  } else {
      meas_noise_ = Eigen::VectorXd::Zero(12);
  }

  // Get current quaternion
  ignition::math::Quaternion<double> gazebo_quaternion =
      ignition::math::Quaternion<double>(gazebo_pose.Rot().W(), gazebo_pose.Rot().X(),
                                 gazebo_pose.Rot().Y(), gazebo_pose.Rot().Z());

  // Convert Ignition quaternion to XYZ fixed angles
  // By default, Ignition orientation are defined as XYZ angles, although called Euler angles
  // (= ZYX Euler angles with X=X, Y=Y, Z=Z, same orientation is obtained by rotating X around fixed X, Y around fixed Y, and Z around fixed Z, as rotating Z around original Z, Y around rotated Y, and X around rotated X)
  ignition::math::Vector3d euler_angles = gazebo_quaternion.Euler();

  // Convert XYZ fixed angles to Ignition quaternion for ground truth state
  ignition::math::Quaternion<double> gazebo_quaternion_state = ignition::math::Quaternion<double>(euler_angles.X(), euler_angles.Y(), euler_angles.Z());

  // Add noise to XYZ fixed angles
  euler_angles.X() += meas_noise_(3); // Roll
  euler_angles.Y() += meas_noise_(4); // Pitch
  euler_angles.Z() += meas_noise_(5); // Yaw

  // Convert XYZ fixed angles to Ignition quaternion for output
  ignition::math::Quaternion<double> gazebo_quaternion_output = ignition::math::Quaternion<double>(euler_angles.X(), euler_angles.Y(), euler_angles.Z());

  // Convert Ignition quaternion to Eigen quaternion
  Eigen::Quaterniond gazebo_quaternion_eigen_state(gazebo_quaternion_state.W(), gazebo_quaternion_state.X(),
                                                   gazebo_quaternion_state.Y(), gazebo_quaternion_state.Z());
  Eigen::Quaterniond gazebo_quaternion_eigen_output(gazebo_quaternion_output.W(), gazebo_quaternion_output.X(),
                                                    gazebo_quaternion_output.Y(), gazebo_quaternion_output.Z());

  // Construct odom state message
  ros_odometry_state_msg_.header.frame_id = parent_frame_id_;
  ros_odometry_state_msg_.header.stamp.sec = (world_->SimTime()).sec;
  ros_odometry_state_msg_.header.stamp.nsec = (world_->SimTime()).nsec;
  ros_odometry_state_msg_.child_frame_id = child_frame_id_;

  // NOTE: linear velocities are given in inertial frame, which is in contrast to the ROS standard!
  ros_odometry_state_msg_.pose.pose.position.x = 
      gazebo_pose.Pos().X();
  ros_odometry_state_msg_.pose.pose.position.y =
      gazebo_pose.Pos().Y();
  ros_odometry_state_msg_.pose.pose.position.z =
      gazebo_pose.Pos().Z();
  ros_odometry_state_msg_.pose.pose.orientation.w = 
      gazebo_quaternion_eigen_state.w();
  ros_odometry_state_msg_.pose.pose.orientation.x =
      gazebo_quaternion_eigen_state.x();
  ros_odometry_state_msg_.pose.pose.orientation.y =
      gazebo_quaternion_eigen_state.y();
  ros_odometry_state_msg_.pose.pose.orientation.z =
      gazebo_quaternion_eigen_state.z();
  ros_odometry_state_msg_.twist.twist.linear.x =
      gazebo_linear_velocity.X();
  ros_odometry_state_msg_.twist.twist.linear.y =
      gazebo_linear_velocity.Y();
  ros_odometry_state_msg_.twist.twist.linear.z =
      gazebo_linear_velocity.Z();
  ros_odometry_state_msg_.twist.twist.angular.x =
      gazebo_angular_velocity.X();
  ros_odometry_state_msg_.twist.twist.angular.y =
      gazebo_angular_velocity.Y();
  ros_odometry_state_msg_.twist.twist.angular.z =
      gazebo_angular_velocity.Z();

  ros_odometry_state_pub_.publish(ros_odometry_state_msg_);

  // Construct odom output message
  ros_odometry_output_msg_.header.frame_id = parent_frame_id_;
  ros_odometry_output_msg_.header.stamp.sec = (world_->SimTime()).sec;
  ros_odometry_output_msg_.header.stamp.nsec = (world_->SimTime()).nsec;
  ros_odometry_output_msg_.child_frame_id = child_frame_id_;

  // NOTE: linear velocities are given in inertial frame, which is in contrast to the ROS standard!
  ros_odometry_output_msg_.pose.pose.position.x = 
      gazebo_pose.Pos().X() + meas_noise_(0);
  ros_odometry_output_msg_.pose.pose.position.y =
      gazebo_pose.Pos().Y() + meas_noise_(1);
  ros_odometry_output_msg_.pose.pose.position.z =
      gazebo_pose.Pos().Z() + meas_noise_(2);
  ros_odometry_output_msg_.pose.pose.orientation.w = 
      gazebo_quaternion_eigen_output.w();
  ros_odometry_output_msg_.pose.pose.orientation.x =
      gazebo_quaternion_eigen_output.x();
  ros_odometry_output_msg_.pose.pose.orientation.y =
      gazebo_quaternion_eigen_output.y();
  ros_odometry_output_msg_.pose.pose.orientation.z =
      gazebo_quaternion_eigen_output.z();
  ros_odometry_output_msg_.twist.twist.linear.x =
      gazebo_linear_velocity.X() + meas_noise_(6);
  ros_odometry_output_msg_.twist.twist.linear.y =
      gazebo_linear_velocity.Y() + meas_noise_(7);
  ros_odometry_output_msg_.twist.twist.linear.z =
      gazebo_linear_velocity.Z() + meas_noise_(8);
  ros_odometry_output_msg_.twist.twist.angular.x =
      gazebo_angular_velocity.X() + meas_noise_(9);
  ros_odometry_output_msg_.twist.twist.angular.y =
      gazebo_angular_velocity.Y() + meas_noise_(10);
  ros_odometry_output_msg_.twist.twist.angular.z =
      gazebo_angular_velocity.Z() + meas_noise_(11);

  ros_odometry_output_pub_.publish(ros_odometry_output_msg_);

  if (add_noise_) {
    meas_noise_msg_.header.frame_id = parent_frame_id_;
    meas_noise_msg_.header.stamp.sec = (world_->SimTime()).sec;
    meas_noise_msg_.header.stamp.nsec = (world_->SimTime()).nsec;
    ros_odometry_output_msg_.child_frame_id = child_frame_id_;

    meas_noise_msg_.y.resize(12);
    for (int i = 0; i < 12; ++i) {
      meas_noise_msg_.y[i] = meas_noise_(i);
    }

    meas_noise_pub_.publish(meas_noise_msg_);
  }

  first_run_ = false;
}

void GazeboOdometryPlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  // ============================================ //
  // =============== POSE MSG SETUP ============= //
  // ============================================ //

  pose_pub_ = node_handle_->Advertise<gazebo::msgs::Pose>(
      "~/" + namespace_ + "/" + pose_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   pose_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                pose_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::POSE);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // == POSE WITH COVARIANCE STAMPED MSG SETUP == //
  // ============================================ //

  pose_with_covariance_stamped_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::PoseWithCovarianceStamped>(
          "~/" + namespace_ + "/" + pose_with_covariance_stamped_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic(
      "~/" + namespace_ + "/" + pose_with_covariance_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(
      namespace_ + "/" + pose_with_covariance_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::POSE_WITH_COVARIANCE_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ========= POSITION STAMPED MSG SETUP ======= //
  // ============================================ //

  position_stamped_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::Vector3dStamped>(
          "~/" + namespace_ + "/" + position_stamped_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   position_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                position_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::VECTOR_3D_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ============= ODOMETRY MSG SETUP =========== //
  // ============================================ //

  // odometry_pub_ = node_handle_->Advertise<gz_geometry_msgs::Odometry>(
  //     "~/" + namespace_ + "/" + odometry_pub_topic_, 5);

  // connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
  //                                                  odometry_pub_topic_);
  // connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
  //                                               odometry_pub_topic_);
  // connect_gazebo_to_ros_topic_msg.set_msgtype(
  //     gz_std_msgs::ConnectGazeboToRosTopic::ODOMETRY);
  // connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
  //                                          true);

  // ============================================ //
  // ======== TRANSFORM STAMPED MSG SETUP ======= //
  // ============================================ //

  transform_stamped_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::TransformStamped>(
          "~/" + namespace_ + "/" + transform_stamped_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic(
      "~/" + namespace_ + "/" + transform_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                transform_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::TRANSFORM_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ===== "BROADCAST TRANSFORM" MSG SETUP =====  //
  // ============================================ //

  broadcast_transform_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::TransformStampedWithFrameIds>(
          "~/" + kBroadcastTransformSubtopic, 1);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboOdometryPlugin);

}  // namespace gazebo
