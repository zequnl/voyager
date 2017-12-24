#include "voyager/planner.hpp"
#include <stdlib.h>

Planner::Planner() {
  // Set running flag
  is_running_ = true;
  // Inform planner is beign initiated
  ROS_INFO_STREAM("Initializing the Planner...");
  // Set the velocities to 0
  vel_pub_ = nh_planner_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  vel_msg_.linear.x = 0;
  vel_msg_.linear.y = 0;
  vel_msg_.linear.z = 0;
  vel_msg_.angular.x = 0;
  vel_msg_.angular.y = 0;
  vel_msg_.angular.z = 0;
  // Publish velocities
  vel_pub_.publish(vel_msg_);
}

Planner::~Planner() {
  // Set running flag to false
  is_running_ = false;
  // Stop the planner
  stop();
}

auto Planner::takeAction() -> void {
  // Check the value of obstacle flag
  if (!laser_scan_.checkCollision()) {
    // Go forward
    vel_msg_.linear.x = 0.5;
    vel_msg_.angular.z = 0.0;
    // Publish velocity
    vel_pub_.publish(vel_msg_);
  } else {
    // Rotate to avoid obstacle
    vel_msg_.linear.x = 0.0;
    vel_msg_.angular.z = -0.3;
    // Publish velocity
    vel_pub_.publish(vel_msg_);
  }
}

auto Planner::isAlive() -> bool {
  // Return the running flag
  return is_running_;
}

auto Planner::setVelocity(float velocity) -> void {
  // Set the vertical velocity
  vel_msg_.linear.z = velocity;
  // Publish the velocity
  vel_pub_.publish(vel_msg_);
}

auto Planner::stop() -> void {
  // Set the velocities to 0
  vel_msg_.linear.x = 0;
  vel_msg_.linear.y = 0;
  vel_msg_.linear.z = 0;
  vel_msg_.angular.x = 0;
  vel_msg_.angular.y = 0;
  vel_msg_.angular.z = 0;
  // Publish the velocity
  vel_pub_.publish(vel_msg_);
}
