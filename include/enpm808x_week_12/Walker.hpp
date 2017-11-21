#pragma once
/**
 * @file walker.hpp
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
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

#define MAX_VAL 0.2
/**
 * @brief      Class for walker.
 */
class Walker {
 public:
  /**
   * @brief      Default Constructor.
   */
  Walker();
  /**
   * @brief      Overloaded Constructor.
   *
   * @param      n_    ROS node handle as ref.
   */
  explicit Walker(ros::NodeHandle n_);
  /**
   * @brief      Destroys the object.
   */
  ~Walker();
  /**
   * @brief      Callback function called upon scan data publish.
   *
   * @param[in]  scan  The scan as sensor_msgs
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

 private:
  /**
   * @brief      Geometry msg for publishing the command.
   */
  geometry_msgs::Twist msgs_;
  /**
   * @brief      Laser scan topic subscriber.
   */
  ros::Subscriber laser_sub_;
  /**
   * @brief      Turtlebot velocity command publisher.
   */
  ros::Publisher velocity_pub_;
};
