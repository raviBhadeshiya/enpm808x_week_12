/**
 * @file walker_node.cpp
 * @author     Ravi Bhadeshiya
 * @version    1.0
 * @brief      Turtlebot walker node
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
#include <ros/ros.h>
#include "enpm808x_week_12/Walker.hpp"
/**
 * @brief      Main function for turtlebot walker node
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     return 0 if everythings works properly
 */
int main(int argc, char** argv) {
  // Init the node
  ros::init(argc, argv, "enpm808x_week_12_node");

  // Node handle created
  ros::NodeHandle n_;

  // Walker walk object created
  Walker walk(n_);

  // Wait for scan msg call back
  ros::spin();

  return 0;
}
