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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);
  float calculateLatencyDistance();
  float calculateLatencyVelocity();
  float calculateFreePathLength(const Eigen::Vector2f& p, float curvature);
  float calculateGoalDist();
  float makeControlDecision();
  float calculateNextVelocity();
  float findClosestObstacle(float curvature);
  float scoreFunction(float curvature);
  struct PathOption pickBestPathOption();

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

 private:

  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

  // stores the TOC phase
  float acceleration_;
  float next_acceleration;

  // latency constant
  const float LATENCY = 0.1;
  const float MAX_VELOCITY = 1.0;
  const float DECELERATION = -4.0;
  const float ACCELERATION = 4.0;

  const float EPSILON = 0.05;
  const float HORIZON = 10.0;

  // car constant
  const float SAFE_MARGIN = 0.1; // TODO: fix me
  const float CAR_LENGTH = 0.4826;  // TODO: fix me
  const float CAR_LENGTH_SAFE = CAR_LENGTH + SAFE_MARGIN * 2;
  const float CAR_BASE = 0.343;  // TODO: fix me
  const float CAR_WIDTH = 0.2667; // TODO: fix me
  const float CAR_WIDTH_SAFE = CAR_WIDTH + SAFE_MARGIN * 2;
  const float LASER_X = 0.2;
  // float MIN_CURVATURE = -1.7857;
  // float MAX_CURVATURE = 1.7857;
  float MIN_CURVATURE = -1.7;
  float MAX_CURVATURE = 1.7;
};

}  // namespace navigation

#endif  // NAVIGATION_H