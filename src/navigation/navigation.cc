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

using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using Eigen::Vector2f;
using std::abs;
using std::cout;
using std::endl;
using std::pow;
using std::sqrt;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace
{
  ros::Publisher drive_pub_;
  ros::Publisher viz_pub_;
  VisualizationMsg local_viz_msg_;
  VisualizationMsg global_viz_msg_;
  AckermannCurvatureDriveMsg drive_msg_;
  // Epsilon value for handling limited numerical precision.
  const float kEpsilon = 1e-4;
} //namespace

namespace navigation
{

  Navigation::Navigation(const string &map_file, ros::NodeHandle *n) : odom_initialized_(false),
                                                                       localization_initialized_(false),
                                                                       robot_loc_(0, 0),
                                                                       robot_angle_(0),
                                                                       robot_vel_(0, 0),
                                                                       robot_omega_(0),
                                                                       nav_complete_(true),
                                                                       nav_goal_loc_(5, 0), // TODO: change to command line argument
                                                                       nav_goal_angle_(0)
  {
    drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
        "ackermann_curvature_drive", 1);
    viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
    local_viz_msg_ = visualization::NewVisualizationMessage(
        "base_link", "navigation_local");
    global_viz_msg_ = visualization::NewVisualizationMessage(
        "map", "navigation_global");
    InitRosHeader("base_link", &drive_msg_.header);
    acceleration_ = 0.0;
  }

  void Navigation::SetNavGoal(const Vector2f &loc, float angle)
  {
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
  }

  void Navigation::UpdateLocation(const Eigen::Vector2f &loc, float angle)
  {
    localization_initialized_ = true;
    robot_loc_ = loc;
    robot_angle_ = angle;
  }

  void Navigation::UpdateOdometry(const Vector2f &loc,
                                  float angle,
                                  const Vector2f &vel,
                                  float ang_vel)
  {
    robot_omega_ = ang_vel;
    robot_vel_ = vel;
    odom_loc_ = loc;
    odom_angle_ = angle;
    if (!odom_initialized_)
    {
      odom_start_angle_ = angle;
      odom_start_loc_ = loc;
      odom_initialized_ = true;
      return;
    }
  }

  void Navigation::ObservePointCloud(const vector<Vector2f> &cloud,
                                     double time)
  {
    cout << "inside ObservePointCloud" << endl;
    point_cloud_ = cloud;
  }

  // calculates how far (distance) the car will travel during the latency
  // takes in the current velocity of the car as well as the current acceleration
  float Navigation::calculateLatencyDistance()
  {
    // uses kinematics to determine how far the robot will move
    // float vnorm = drive_msg_.velocity;
    float final_v = calculateLatencyVelocity();
    // assume the car is constantly accelerating
    return 0.5 * (drive_msg_.velocity + final_v) * LATENCY;
  }

  // calculates what the velocity will be after the latency period, capping it at max
  float Navigation::calculateLatencyVelocity()
  {
    float vnorm = drive_msg_.velocity;
    float final_velocity = vnorm + acceleration_ * LATENCY < MAX_VELOCITY ? vnorm + acceleration_ * LATENCY : MAX_VELOCITY;
    // cout << "acceleration_: " << acceleration_ << endl;
    return final_velocity > 0 ? final_velocity : 0;
  }

  // calculates the distance remaining to the target along a fixed arc
  float Navigation::calculateFreePathLength(const Eigen::Vector2f &p, float curvature)
  {
    // TODO: fix me. assuming p is in the map fram
    float x = p.x() - LASER_X;
    float y = p.y();

    if (curvature <= kEpsilon && curvature >= -kEpsilon)
    {
      // check if the goal is in front of the car
      if (y <= CAR_WIDTH_SAFE && y >= -CAR_WIDTH_SAFE && x >= (CAR_LENGTH_SAFE + CAR_BASE) / 2)
        return x - (CAR_LENGTH_SAFE + CAR_BASE) / 2 >= 0 ? x - (CAR_LENGTH_SAFE + CAR_BASE) / 2 : 0;
      else
        return HORIZON;
    }

    // distance from base_link frame origin to center of turning
    float r_c = 1 / curvature;
    r_c = r_c > 0 ? r_c : -r_c;
    // distance from center of turning to the goal
    float r_goal = sqrt(pow(x, 2) + pow((r_c - y), 2));

    // distance from center of turning to the car
    float r_inner_back = r_c - CAR_WIDTH_SAFE / 2;
    float r_inner_front = 0.5 * sqrt(pow(2 * r_c - CAR_WIDTH_SAFE, 2) + pow(CAR_BASE + CAR_LENGTH_SAFE, 2));
    float r_outer_front = 0.5 * sqrt(pow(2 * r_c + CAR_WIDTH_SAFE, 2) + pow(CAR_BASE + CAR_LENGTH_SAFE, 2));

    bool hit_side = r_goal >= r_inner_back && r_goal <= r_inner_front;
    bool hit_front = r_goal >= r_inner_front && r_goal <= r_outer_front;

    float theta = 0.0;
    if (hit_front)
    {
      theta = asin(x / r_goal) - asin((CAR_LENGTH_SAFE + CAR_BASE) / 2 / r_goal);
    }
    else if (hit_side)
    {
      theta = asin(x / r_goal) - acos((r_c - CAR_WIDTH_SAFE / 2) / r_goal);
    }
    else
    {                   // will not hit obstacle
      theta = 2 * M_PI; // upper bound
    }
    return theta * r_c;
  }

  // float Navigation::calculateGoalDist() {
  //   float x = nav_goal_loc_.x() - robot_loc_.x();
  //   float y = nav_goal_loc_.y() - robot_loc_.y();
    
  //   // TODO: fix me. assume goal is on the arc the car is traveling
  //   if (drive_msg_.curvature == 0) {
  //     if (y <= EPSILON && y >= -EPSILON && x >= EPSILON ) {
  //       return x;
  //     } else {
  //       return 0; // TODO: fix me
  //     }
  //   }
  //   float r_c = 1 / drive_msg_.curvature;
  //   r_c = r_c > 0 ? r_c : -r_c;
  //   float theta = asin(x/r_c);
  //   return theta * r_c;
  // }

  float Navigation::findClosestObstacle(float curvature) {
    float min_path_len = HORIZON;
    for (Eigen::Vector2f v : point_cloud_) {
      // calculate free path length for each point
      float path_len = calculateFreePathLength(v, curvature);

      // cout << "path_len: " << path_len << endl;
      if (path_len < min_path_len) {
        min_path_len = path_len;
      }
    }
    // return the minimum value
    return min_path_len;
  }

  /*
    freePathLength: 0-HORIZON
    clearance:
    goalDist: 0-HORIZON
  */
  float Navigation::scoreFunction(float curvature) {
    //const float INTERVAL = 0.05;
    //float current_goal_dist = HORIZON;
    float w_clearance = 0.0;
    float w_goal_dist = 3.0;
    float clearance = 0.0;
    float free_path_length = findClosestObstacle(curvature);
    //float travel_distance = (calculateLatencyVelocity() + calculateNextVelocity()) / 2 * INTERVAL;
    
    // float next_goal_dist;
    // if(curvature == 0) {
    //   next_goal_dist = current_goal_dist - travel_distance;
    // } else {
    //   float angle = travel_distance * curvature;
    //   float x = sin(angle) / curvature;
    //   float y = (1 - cos(angle)) / curvature;
    //   next_goal_dist = sqrt(pow(current_goal_dist - x, 2) + pow(y, 2));
    // }
    
    //return free_path_length +  w_clearance * clearance + w_goal_dist * (HORIZON - next_goal_dist);
    return free_path_length +  w_clearance * clearance + w_goal_dist * (MAX_CURVATURE - curvature);

  }

  /* 
   * check all possible curvatures
   * store remaining_dist for each curvature
   */ 
  struct PathOption Navigation::pickBestPathOption() {
    // float MIN_CURVATURE = -1.0 / CAR_WIDTH_SAFE; // TODO: fix me
    // float MAX_CURVATURE = 1.0 / CAR_WIDTH_SAFE; // TODO: fix me


    struct PathOption best_path = {0, 0, 0, Vector2f(0,0), Vector2f(0,0)};
    const float CURVATURE_STEP = 0.1;

    float best_score = -100000.0;
    for (float c = MIN_CURVATURE; c <= MAX_CURVATURE; c += CURVATURE_STEP) {
      visualization::DrawPathOption(c, findClosestObstacle(c), 0, local_viz_msg_);
      float score = scoreFunction(c);
      if (score > best_score) {
        best_path.curvature = c;
        best_path.clearance = 0;
        best_path.free_path_length = findClosestObstacle(c);
        best_score = score;
      }
    }
    return best_path;
  }

  // Decides whether to accelerate (4.0), decelerate (-4), or maintain velocity (0)
  float Navigation::makeControlDecision()
  {
    // Pick best path option here
    struct PathOption best_path = pickBestPathOption();
    
    float curr_velocity = calculateLatencyVelocity();
    float remaining_dist = best_path.free_path_length - calculateLatencyDistance();
    float stopping_dist = -1 * pow(curr_velocity, 2) / (2 * DECELERATION);

    cout << "stopping_dist: " << stopping_dist << ", remaining_dist: " << remaining_dist
         << ", Obstacle: " << best_path.free_path_length << ", latency dist: " << calculateLatencyDistance() << ", velocity: " << curr_velocity << endl;
    if (stopping_dist >= remaining_dist)
    {
      acceleration_ = DECELERATION;
    }
    else if (abs(stopping_dist - remaining_dist) > kEpsilon && curr_velocity < MAX_VELOCITY)
    {
      acceleration_ = ACCELERATION;
    }
    else
    {
      acceleration_ = 0;
    }

    return best_path.curvature;
  }

  // takes in the acceleration determined by makeControlDecision() and the current
  // velocity to figure out what the velocity should be by the next time stamp
  float Navigation::calculateNextVelocity()
  {
    float velocity = calculateLatencyVelocity();
    float final_velocity = velocity + acceleration_ * 0.05 < MAX_VELOCITY ? velocity + acceleration_ * 0.05 : MAX_VELOCITY;
    // cout << "final_velocity " << final_velocity << endl;
    return final_velocity > 0 ? final_velocity : 0;
  }  

  void Navigation::Run()
  {
    // This function gets called 20 times a second to form the control loop.

    // Clear previous visualizations.
    visualization::ClearVisualizationMsg(local_viz_msg_);
    visualization::ClearVisualizationMsg(global_viz_msg_);

    // If odometry has not been initialized, we can't do anything.
    if (!odom_initialized_)
      return;
    // The control iteration goes here.
    // Feel free to make helper functions to structure the control appropriately.

      drive_msg_.curvature = makeControlDecision();
      // cout << "acceleration" << acceleration_ << endl;
      drive_msg_.velocity = calculateNextVelocity();
      // cout << "drive_msg_.velocity " << drive_msg_.velocity << endl;
      visualization::DrawPathOption(drive_msg_.curvature, 1, 0, local_viz_msg_);


    // The latest observed point cloud is accessible via "point_cloud_"

    // Eventually, you will have to set the control values to issue drive commands:
    // drive_msg_.curvature = 0;
    // drive_msg_.velocity = 1;

    // Add timestamps to all messages.
    local_viz_msg_.header.stamp = ros::Time::now();
    global_viz_msg_.header.stamp = ros::Time::now();
    drive_msg_.header.stamp = ros::Time::now();
    // Publish messages.
    viz_pub_.publish(local_viz_msg_);
    viz_pub_.publish(global_viz_msg_);
    drive_pub_.publish(drive_msg_);
  }

} // namespace navigation

/**
1. How to convert the LaserScan into the point cloud
2. How precisely do we have to hit the navigation target?
3. How to handle scoring function with values of different magnitudes? Normalization?
**/
