/**
 * @file walker.cpp
 * @author     Ravi Bhadeshiya
 * @version    1.0
 * @brief      Turtlebot walker class
 *
 * @copyright  MIT License (c) 2017 Ravi Bhadeshiya
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "enpm808x_week_12/Walker.hpp"

// Constructor
Walker::Walker() {}

// Overloaded Constructor
Walker::Walker(ros::NodeHandle n_) {
  velocity_pub_ =
      n_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 30);
  laser_sub_ = n_.subscribe("/scan", 30, &Walker::laserCallback, this);

  // Reset the the velocity
  msgs_.linear.x = 0;
  msgs_.angular.z = 0;
  velocity_pub_.publish(msgs_);

  ROS_INFO("Turtlebot walker is init successfully..");
}

// Destructor
Walker::~Walker() {
  ROS_INFO("Turtlebot walker is shutting down..");

  // On Destruction stop the robot
  msgs_.linear.x = 0;
  msgs_.angular.z = 0;
  velocity_pub_.publish(msgs_);
}

void Walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  ROS_DEBUG("LaserCallback called!");
  // If obst is nearby go to turning behavior
  for (const auto& itr : scan->ranges) {
    // scan->range_min is 0.45
    if (itr <= scan->range_min + 0.5) {
      msgs_.linear.x = 0;
      msgs_.angular.z = MAX_VAL;
      velocity_pub_.publish(msgs_);
      ROS_WARN("Obst detected!--> Turnning..");
      return;
    }
  }
  // Else walk straight
  msgs_.linear.x = MAX_VAL;
  msgs_.angular.z = 0;
  velocity_pub_.publish(msgs_);
  ROS_DEBUG("Straight..");
  return;
}
