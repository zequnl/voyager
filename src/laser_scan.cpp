#include "voyager/laser_scan.hpp"
#include <vector>

LaserScan::LaserScan() {
  // Set the running flag to true
  is_running_ = true;
  // Set the obstacle flag to false
  obstacle_flag_ = false;
  // Create a subscriber
  scan_sub_ = nh_laser_scan_.subscribe<sensor_msgs::LaserScan>(
      "/scan", 1000, &LaserScan::scanCallback, this);
}

LaserScan::~LaserScan() {
  // Set the running flag to false
  is_running_ = false;
  // Set the obstacle flag to false
  obstacle_flag_ = false;
}

auto LaserScan::scanCallback(const sensor_msgs::LaserScan scan_msg) -> void {
  // Create a vector from the messages
  std::vector<float> ranges = scan_msg.ranges;
  // Get the size of the array
  size_t size = ranges.size();
  // Find the center of the array
  int center = size / 2;
  // Determing what range should be checked
  float angleCheck = 30 * 3.14 / 180;
  float increment = scan_msg.angle_increment;
  int range = round(angleCheck / increment);
  // Find the left most value and right most value
  std::vector<float>::const_iterator minRange = ranges.begin() + center - range;
  std::vector<float>::const_iterator maxRange = ranges.begin() + center + range;
  // Create a new vector with pruned messages
  std::vector<float> choppedRanges(minRange, maxRange);
  // Set obstacle flag to false
  obstacle_flag_ = false;

  for (auto i : choppedRanges) {
    // Check if the distance is less than threshold
    if (i < 3) {
      // Set the collision flag to true
      obstacle_flag_ = true;
    }
  }
}

auto LaserScan::isAlive() -> bool {
  // Return running flag
  return is_running_;
}

auto LaserScan::checkCollision() -> bool {
  // Return obstacle flag
  return obstacle_flag_;
}
