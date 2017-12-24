#ifndef INCLUDE_VOYAGER_QUADROTOR_HPP_
#define INCLUDE_VOYAGER_QUADROTOR_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include "voyager/explore.h"
#include "voyager/planner.hpp"

class Quadrotor {
 public:
  Quadrotor();
  ~Quadrotor();
  auto exploreAndMap() -> void;
  auto isAlive() -> bool;
  auto heightCallback(const sensor_msgs::Range height_msg) -> void;
  auto getHeight() -> float;
  auto explore(voyager::explore::Request& request,
               voyager::explore::Response& resp) -> bool;
  auto getExplorerFlag() -> bool;
  auto stop() -> void;

 private:
  Planner planner_;             ///< Instance of Planner class
  bool is_running_;             ///< Flag to check if running
  ros::NodeHandle nh_quad_;     ///< Nodehandle for the quadrotor class
  float height_;                ///< Height of the robot from ground
  ros::Subscriber height_sub_;  ///< Subscriber for the topic /sonar_height
  bool explore_flag_;           ///< Flag to indicate the user input
  ros::ServiceServer server_;   ///< Service server for /explore
};

#endif  // INCLUDE_VOYAGER_QUADROTOR_HPP_
