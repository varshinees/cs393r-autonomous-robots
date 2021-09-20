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
  const float kEpsilon = 1e-3;
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
    point_cloud_ = cloud;
  }

  float norm(float x, float y) {
    return sqrt(pow(x, 2) + pow(y, 2));
  }

  float Navigation::getLatencyVelocity()
  {
    float initial_v = norm(robot_vel_.x(), robot_vel_.y());
    // float initial_v = drive_msg_.velocity;
    float final_v = initial_v + acceleration_ * LATENCY;
    return final_v < MAX_VELOCITY ? (final_v > 0 ? final_v : 0) : MAX_VELOCITY;
  }

  float Navigation::getNextVelocity(float acceleration)
  {
    float initial_v = getLatencyVelocity();
    float final_v = initial_v + acceleration * INTERVAL;
    return final_v < MAX_VELOCITY ? final_v > 0 ? final_v : 0 : MAX_VELOCITY;
  }  

  float Navigation::getLatencyDistance()
  {
    // Assume the car is constantly accelerating
    float initial_v = norm(robot_vel_.x(), robot_vel_.y());
    // float initial_v = drive_msg_.velocity;
    float final_v = getLatencyVelocity();
    return 0.5 * (initial_v + final_v) * LATENCY;
  }

  float Navigation::getFreePathLength(const Eigen::Vector2f &p, float curvature)
  {
    // if there's no obstacle, the LIDAR returns its limit
    if (norm(p.x(), p.y()) >= HORIZON - kEpsilon)
      return HORIZON;
    
    // Tranform p to from the laser frame to the base_link frame
    float x = p.x();
    float y = p.y();

    // The car is going straight
    if (abs(curvature) <= kEpsilon) {
      // check if the goal is in front of the car
      if (y <= CAR_WIDTH_SAFE && y >= -CAR_WIDTH_SAFE && x >= (CAR_LENGTH_SAFE + CAR_BASE) / 2) {
        float free_path_length = x - (CAR_LENGTH_SAFE + CAR_BASE) / 2;
        return free_path_length > 0 ? free_path_length : 0;
      } else {
        return HORIZON;
      }
    }

    // Radius of turning
    float r_c = 1 / curvature;
    // Distance from center of turning to p
    float r_p = norm(x, r_c - y);
    r_c = abs(r_c);

    // Distance from center of turning to the car
    float r_inner_back = r_c - CAR_WIDTH_SAFE / 2;
    float r_inner_front = 0.5 * norm(2 * r_c - CAR_WIDTH_SAFE, CAR_BASE + CAR_LENGTH_SAFE);
    float r_outer_front = 0.5 * norm(2 * r_c + CAR_WIDTH_SAFE, CAR_BASE + CAR_LENGTH_SAFE);

    bool hit_side = r_p >= r_inner_back && r_p <= r_inner_front;
    bool hit_front = r_p >= r_inner_front && r_p <= r_outer_front;

    // Calculate the center angle of the free_path arc
    float alpha = asin(x / r_p);
    float beta;
    if (hit_front) {
      beta = asin((CAR_LENGTH_SAFE + CAR_BASE) / 2 / r_p);
    } else if (hit_side) {
      beta = acos((r_c - CAR_WIDTH_SAFE / 2) / r_p);
    } 
    float theta = 0.0;
    if (hit_front || hit_side) {
      if (x >= 0.0 && abs(y) <= r_c )
        theta = alpha - beta;
      else if (x >= 0.0 && abs(y) > r_c)
        theta = M_PI - alpha - beta;
      else if (x < 0.0 && abs(y) > r_c)
        theta = M_PI - alpha - beta; // alpha < 0
      else
        theta = 2 * M_PI + alpha - beta; // alpha < 0
    } else {
      theta = 2 * M_PI;
    }
    theta = theta > 0 ? theta : 0;

    return theta * r_c > HORIZON ? HORIZON : theta * r_c;
  }

  float Navigation::getClosestObstacleDistance(float curvature) {
    float min_path_len = HORIZON;
    for (Eigen::Vector2f v : point_cloud_) {
      float path_len = getFreePathLength(v, curvature);
      min_path_len = path_len < min_path_len ? path_len : min_path_len;
    }
    return min_path_len;
  }

  // float Navigation::getGoalDist() {
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

  float Navigation::getClearance(float curvature, const Eigen::Vector2f &p, float free_path_length) {
    // if there's no obstacle, the LIDAR returns its limit
    if (norm(p.x(), p.y()) >= HORIZON - kEpsilon)
      return HORIZON;

    float x = p.x();
    float y = p.y();
    
    if(abs(curvature) <= kEpsilon) {
      if (x >= (CAR_LENGTH_SAFE + CAR_BASE) / 2 && x < (CAR_LENGTH_SAFE + CAR_BASE) / 2 + free_path_length - kEpsilon) {
        float clearance = abs(y) - CAR_WIDTH_SAFE / 2;
        return clearance > 0 ? clearance : 0;
      } else {
        return HORIZON;
      }
    }

    float bounding_angle = abs(free_path_length * curvature) - kEpsilon;
    bounding_angle = bounding_angle > 0 ? bounding_angle : 0;
    float r_c = 1 / curvature;
    // Distance from center of turning to p
    float r_p = norm(x, r_c - y);

    float theta = 0;
    float alpha = asin(x / r_p);
    if(x > 0) {
      if ((r_c >= 0 && p.y() < r_c) || (r_c < 0 && p.y() >= r_c)) {
        theta = alpha;
      } else {
        theta = M_PI - alpha;
      }
    } else {
      if ((r_c < 0 && p.y() < r_c) || (r_c >= 0 && p.y() >= r_c)) {
        theta = M_PI - alpha;
      } else {
        theta = 2 * M_PI + alpha;
      }
    }
    r_c = abs(r_c);
    theta -= asin((CAR_LENGTH_SAFE + CAR_BASE) / 2 / r_c) + kEpsilon;

    // Distance from center of turning to the car
    float r_inner_back = r_c - CAR_WIDTH_SAFE / 2;
    float r_outer_front = 0.5 * norm(2 * r_c + CAR_WIDTH_SAFE, CAR_BASE + CAR_LENGTH_SAFE);

    if (theta > 0 && theta < bounding_angle) {
      if (r_p < r_inner_back)  // p is on the inside of the car's arc
        return r_inner_back - r_p;
      else if (r_p > r_outer_front)  // p is on the outside of the car's arc
        return r_p - r_outer_front;
      // No points should lay between r_inner_back and r_outer_front since it's bounded by the free path length
    } 
    // p isn't in the path of the car at all
    return -1.0;
  }

  float Navigation::getMinClearance(float curvature, float free_path_length) {
    if(free_path_length < kEpsilon)
      return 0.0;

    float min_clearance = HORIZON;
    for (Eigen::Vector2f v : point_cloud_) {
      float clearance = getClearance(curvature, v, free_path_length);
      if (clearance < 0.0) continue;
      min_clearance = clearance < min_clearance ? clearance : min_clearance;
    }
    return min_clearance;
  }

  float Navigation::getScore(float curvature, struct PathOption &path) {
    float w_clearance = 0.3;
    float w_goal_dist = 1.5;
    
    float free_path_length = getClosestObstacleDistance(curvature);
    float clearance = getMinClearance(curvature, free_path_length);

    // float current_goal_dist = HORIZON;
    // float travel_distance = (getLatencyVelocity() + getNextVelocity()) / 2 * INTERVAL;
    // float next_goal_dist;
    // if(curvature == 0) {
    //   next_goal_dist = current_goal_dist - travel_distance;
    // } else {
    //   float angle = travel_distance * curvature;
    //   float x = sin(angle) / curvature;
    //   float y = (1 - cos(angle)) / curvature;
    //   next_goal_dist = norm(current_goal_dist - x, y);
    // }
    
    path.curvature = curvature;
    path.free_path_length = free_path_length;
    path.clearance = clearance;
    
    //return free_path_length +  w_clearance * clearance + w_goal_dist * (HORIZON - next_goal_dist);
    return free_path_length + w_clearance * clearance + w_goal_dist * (MAX_CURVATURE - abs(curvature));
  }

  struct PathOption Navigation::getBestPathOption() {
    struct PathOption best_path = {0, 0, 0, Vector2f(0,0), Vector2f(0,0)};
    struct PathOption path = {0, 0, 0, Vector2f(0,0), Vector2f(0,0)};

    const float CURVATURE_STEP = 0.05;
    float best_score = -100000.0;
    for (float c = MIN_CURVATURE; c <= MAX_CURVATURE; c += CURVATURE_STEP) {
      float score = getScore(c, path);
      visualization::DrawPathOption(c, path.free_path_length, 0, local_viz_msg_);
      if (score > best_score) {
        best_path.curvature = path.curvature;
        best_path.free_path_length = path.free_path_length;
        // TODO: best_path.clearance is just an approximation now
        best_path.clearance = path.clearance + CAR_WIDTH_SAFE / 2;
        best_score = score;
      }
    }
    visualization::DrawPathOption(best_path.curvature, best_path.free_path_length, best_path.clearance, local_viz_msg_);
    return best_path;
  }

  void Navigation::makeControlDecision()
  {
    struct PathOption best_path = getBestPathOption();
    
    float curr_velocity = getLatencyVelocity();
    float remaining_dist = best_path.free_path_length - getLatencyDistance();
    remaining_dist = remaining_dist > 0 ? remaining_dist : 0;
    float decelerate_dist = -1 * pow(curr_velocity, 2) / (2 * DECELERATION);

    float accelerate_final_v = getNextVelocity(ACCELERATION);
    float accelerate_dist = 0.5 * (curr_velocity + accelerate_final_v) * INTERVAL - 1 * pow(accelerate_final_v, 2) / (2 * DECELERATION);
    
    float const_dist = decelerate_dist + curr_velocity * INTERVAL;
   
    // Decides whether to accelerate (4.0), decelerate (-4), or maintain velocity (0)
    float next_acceleration;
    if(remaining_dist < 0.02)
      next_acceleration = DECELERATION;
    else if (accelerate_dist < remaining_dist - kEpsilon && curr_velocity < MAX_VELOCITY)
      next_acceleration = ACCELERATION;
    else if (const_dist < remaining_dist - kEpsilon)
      next_acceleration = 0;
    else
      next_acceleration = DECELERATION;
    
    // printf("remaining_dist: %f, free_path: %f, latency_dist: %f, accelerate_dist: %.2f, const_dist: %.2f\n", 
    //   remaining_dist, best_path.free_path_length, getLatencyDistance(), accelerate_dist, const_dist);
    // printf("vnorm: %.2f, drive_vel_: %.2f, curr_velocity: %f, acceleration_: %.2f, next_acceleration: %.2f\n\n", 
    //   norm(robot_vel_.x(), robot_vel_.y()), drive_msg_.velocity, curr_velocity, acceleration_, next_acceleration);

    drive_msg_.curvature = best_path.curvature;
    drive_msg_.velocity = getNextVelocity(next_acceleration);
    acceleration_ = next_acceleration;
  }

  void Navigation::Run()
  {
    // This function gets called 20 times a second to form the control loop.

    // Clear previous visualizations.
    visualization::ClearVisualizationMsg(local_viz_msg_);
    visualization::ClearVisualizationMsg(global_viz_msg_);

    drawVisualizations();

    // If odometry has not been initialized, we can't do anything.
    if (!odom_initialized_)
      return;

    makeControlDecision();

    // Add timestamps to all messages.
    local_viz_msg_.header.stamp = ros::Time::now();
    global_viz_msg_.header.stamp = ros::Time::now();
    drive_msg_.header.stamp = ros::Time::now();
    // Publish messages.
    viz_pub_.publish(local_viz_msg_);
    viz_pub_.publish(global_viz_msg_);
    drive_pub_.publish(drive_msg_);
  }

  void Navigation::drawVisualizations() {
    // car left side
    visualization::DrawLine(Eigen::Vector2f((CAR_BASE - CAR_LENGTH_SAFE)/2, CAR_WIDTH_SAFE/2),
                            Eigen::Vector2f((CAR_BASE - CAR_LENGTH_SAFE)/2, -CAR_WIDTH_SAFE/2),
                            0x000000,
                            local_viz_msg_);
    // car right side
    visualization::DrawLine(Eigen::Vector2f((CAR_BASE + CAR_LENGTH_SAFE)/2, CAR_WIDTH_SAFE/2),
              Eigen::Vector2f((CAR_BASE + CAR_LENGTH_SAFE)/2, -CAR_WIDTH_SAFE/2),
              0x000000,
              local_viz_msg_);
    // car up side
    visualization::DrawLine(Eigen::Vector2f((CAR_BASE - CAR_LENGTH_SAFE)/2, CAR_WIDTH_SAFE/2),
                            Eigen::Vector2f((CAR_BASE + CAR_LENGTH_SAFE)/2, CAR_WIDTH_SAFE/2),
                            0x000000,
                            local_viz_msg_);
    // car down side
    visualization::DrawLine(Eigen::Vector2f((CAR_BASE - CAR_LENGTH_SAFE)/2, -CAR_WIDTH_SAFE/2),
                            Eigen::Vector2f((CAR_BASE + CAR_LENGTH_SAFE)/2, -CAR_WIDTH_SAFE/2),
                            0x000000,
                            local_viz_msg_);

    // draw point cloud
    // for (Eigen::Vector2f v : point_cloud_) {
    //   Eigen::Vector2f vprime(v.x(), v.y());
    //   visualization::DrawPoint(vprime, 0xff00d4, local_viz_msg_);
    // }
  }

} // namespace navigation

