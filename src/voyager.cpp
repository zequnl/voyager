
/**
 * @file    voyager.cpp
 * @author  zequnl
 * @brief DESCRIPTION
 * Main file for the package voyager
 *
 */

#include <hector_uav_msgs/EnableMotors.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "voyager/explore.h"
#include "voyager/quadrotor.hpp"

int main(int argc, char** argv) {
  // Initialize the node
  ros::init(argc, argv, "voyager_node");
  // Create a nodehandle
  ros::NodeHandle nh;
  // Inform the user
  ROS_INFO_STREAM("Starting Voyager node...");
  // Create a client for service /explore
  ros::ServiceClient exploreClient =
      nh.serviceClient<voyager::explore>("explore");
  // Create a client for service /enable_motors
  ros::ServiceClient motorsClient =
      nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
  // Create a service instance
  hector_uav_msgs::EnableMotors srv;
  // Set the request
  srv.request.enable = true;
  // Call the service
  motorsClient.call(srv);
  // Wait for service to advertise
  if (ros::service::waitForService("explore", 1000)) {
    ROS_INFO_STREAM("The service is available!");
  }
  // Create an instance of the quadrotor
  Quadrotor quad;
  // Check rosmaster
  while (ros::ok()) {
    // Check the user input
    if (quad.getExplorerFlag()) {
      // Explore the map
      quad.exploreAndMap();
    } else {
      // Stay at the same place
      quad.stop();
    }
    // Spin in callbacks
    ros::spinOnce();
  }
  // Disable motors
  srv.request.enable = false;
  // Call the service
  motorsClient.call(srv);

  return 0;
}
