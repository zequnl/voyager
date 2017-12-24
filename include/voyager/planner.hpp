#ifndef INCLUDE_VOYAGER_PLANNER_HPP_
#define INCLUDE_VOYAGER_PLANNER_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "voyager/laser_scan.hpp"

class Planner {
 public:
  Planner();
  ~Planner();
  auto takeAction() -> void;
  auto isAlive() -> bool;
  auto setVelocity(float velocity) -> void;
  auto stop() -> void;

 private:
  LaserScan laser_scan_;          ///< Instance of LaserScan class
  bool is_running_;               ///< Flag to check if running
  ros::NodeHandle nh_planner_;    ///< Nodehandle for the planner class
  geometry_msgs::Twist vel_msg_;  ///< Message over which data will be published
  ros::Publisher vel_pub_;        ///< Publisher for publishing velocity
};

#endif  // INCLUDE_VOYAGER_PLANNER_HPP_
