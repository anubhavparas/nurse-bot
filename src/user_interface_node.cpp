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
#include <nurse_bot/Task.h>
#include <nurse_bot/NBTaskAction.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <memory>
#include <sstream>

#include <nurse-bot/task_publisher.hpp>
#include <nurse-bot/task_action_client.hpp>


int main(int argc, char **argv) {
  ros::init(argc, argv, "user_interface_node");

  ROS_WARN_STREAM("Waiting for the arm to be tucked in....");
  ros::WallDuration(60.0).sleep();
  ROS_WARN_STREAM("Arm mgiht be tucked in");

  ros::NodeHandle ros_node_h;
  std::unique_ptr<nursebot::TaskPublisher> task_pub(
                        new nursebot::TaskPublisher());

  nurse_bot::Task task_msg;
  task_msg.task_id = "G1";
  // task_pub->publish(task_msg);



  std::unique_ptr<nursebot::TaskActionClient> task_ac(
                new nursebot::TaskActionClient("nursebot_actionserver"));

  nurse_bot::NBTaskGoal task_goal;
  task_goal.task_id = "G1";
  task_goal.task_type = "Guidance";

  task_ac->request_action(task_goal);

  ros::spin();
  return 0;
}
