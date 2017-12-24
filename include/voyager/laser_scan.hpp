#ifndef INCLUDE_VOYAGER_LASER_SCAN_HPP_
#define INCLUDE_VOYAGER_LASER_SCAN_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LaserScan {
 public:
  LaserScan();
  ~LaserScan();
  auto scanCallback(const sensor_msgs::LaserScan scan_msg) -> void;
  auto isAlive() -> bool;
  auto checkCollision() -> bool;

 private:
  ros::NodeHandle nh_laser_scan_;  ///< Node handle for this class
  bool is_running_;                ///< Flag to indicate if the class is running
  bool obstacle_flag_;             ///< Flag to indicate if obstacle is detected
  ros::Subscriber scan_sub_;       ///< Subscriber that can be used to listen to
                                   ///< LaserScan messages
};

#endif  // INCLUDE_VOYAGER_LASER_SCAN_HPP_
