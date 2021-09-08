//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include <cmath>

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using std::sqrt;
using std::pow;
using std::abs;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  odom_loc_ = loc;
  odom_angle_ = angle;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    return;
  }
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

// calculates how far (distance) the car will travel during the latency
// takes in the current velocity of the car as well as the current acceleration
float Navigation::calculateLatencyDistance() {
  // uses kinematics to determine how far the robot will move 
  float vnorm = sqrt(pow(robot_vel_.x(), 2) + pow(robot_vel_.y(), 2));
  float final_v = calculateLatencyVelocity();
  // assume the car is constantly accelerating
  return 0.5 * (vnorm + final_v) * LATENCY;
}

// calculates what the velocity will be after the latency period, capping it at max
float Navigation::calculateLatencyVelocity() {
  // uses kinematics
  float vnorm = sqrt(pow(robot_vel_.x(), 2) + pow(robot_vel_.y(), 2));
  float final_velocity = std::min(vnorm + acceleration_ * LATENCY, MAX_VELOCITY);
  return final_velocity > 0 ? final_velocity : 0;
}

// calculates the distance remaining to the target along a fixed arc
float Navigation::calculateRemainingDistance() {
  float travelled_dist = calculateLatencyDistance();
  // remember to subtract out the latency distance before returning!
}

// Decides whether to accelerate (4.0), decelerate (-4), or maintain velocity (0)
void Navigation::makeControlDecision() {
  /*
    Pseudocode (1D TOC):
      calculateLatencyDistance()
      transform robot's current location along the current arc by that distance
      calculate remaining distance to destination
      calculate stopping distance based on current velocity & acceleration
      **NOTE: "current velocity" refers to the velocity at the end of the latency period

      if stopping distance >= remaining distance
        decelerate
      if stopping distance < remaining distance && current velocity < max velocity
        accelerate
      else 
        maintain speed

      acceleration_ = decision (4, -4, or 0)
  */

  float curr_velocity = calculateLatencyVelocity();

  // calculate remaining distance here
  float remaining_dist = calculateRemainingDistance();

  const float DECELERATION = -4.0;
  const float ACCELERATION = 4.0;

  float stopping_dist = -1 * pow(curr_velocity, 2) / (2 * DECELERATION);

  if (stopping_dist >= remaining_dist) {
    acceleration_ = DECELERATION;
  } else if (stopping_dist < remaining_dist && curr_velocity < MAX_VELOCITY) {
    acceleration_ = ACCELERATION;
  } else {
    acceleration_ = 0;
  }
  
}

// takes in the acceleration determined by makeControlDecision() and the current 
// velocity to figure out what the velocity should be by the next time stamp
float Navigation::calculateNextVelocity() {
  // basic kinematics here
  float velocity = calculateLatencyVelocity();
  float final_velocity = std::min(velocity + acceleration_ * 0.05, MAX_VELOCITY);
  return final_velocity > 0 ? final_velocity : 0; 
  
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  makeControlDecision();
  drive_msg_.velocity = calculateNextVelocity();
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  drive_msg_.curvature = 0;
  drive_msg_.velocity = 1;

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation

/**
1. to get curvature: drive_msg_ vs robot_angle?
2.nav_goal_angle_: why we need it?

**/
