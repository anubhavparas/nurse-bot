/**
 * MIT License
 *
 * Copyright (c) 2021 Sakshi Kakde, Siddharth Telang, Anubhav Paras
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file user_interface_node.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief ROS node for to spawn user interface and publish messages
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include<nurse_bot/Task.h>

#include <iostream>
#include <memory>
#include <sstream>

#include <nurse-bot/task_publisher.hpp>


int main(int argc, char **argv) {
  ros::init(argc, argv, "user_interface_node");
  ros::NodeHandle ros_node_h;
  std::unique_ptr<nursebot::TaskPublisher> task_pub(
                        new nursebot::TaskPublisher());

  nurse_bot::Task task_msg;
  task_pub->publish(task_msg);

//   std::shared_ptr<MoveBaseActionWrapper> movebase_action =
//                       std::make_shared<MoveBaseActionWrapper>("map", true);
//   std::shared_ptr<Navigator> navigator =
//               std::make_shared<MapNavigator>(movebase_action);
//   this->task_action = std::make_shared<nursebot::GuidanceTask>(navigator);

//   this->task_action->perform_task(task_msg);


  ros::spin();
  return 0;
}
