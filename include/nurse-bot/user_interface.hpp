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

#ifndef INCLUDE_NURSE_BOT_USER_INTERFACE_HPP_
#define INCLUDE_NURSE_BOT_USER_INTERFACE_HPP_

#include <geometry_msgs/Twist.h>
#include <nurse_bot/NBTaskAction.h>
#include <nurse_bot/Task.h>
#include <ros/ros.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>
#include <nurse-bot/task_action_client.hpp>
#include <nurse-bot/task_publisher.hpp>

namespace nursebot {

class UserInterface {
 public:
  /**
   * @brief Construct a new UserInterface object
   * 
   * @param _task_ac 
   */
  explicit UserInterface(
      const std::shared_ptr<nursebot::TaskActionClient>& _task_ac);

  /**
   * @brief Destroy the UserInterface object
   * 
   */
  virtual ~UserInterface();

  /**
   * @brief method to ask for user inputs for entity location and target location
   * 
   */
  void get_user_input();

 private:
  std::shared_ptr<nursebot::TaskActionClient> task_ac;
  geometry_msgs::Twist entity_position;
  geometry_msgs::Twist target_position;
  nurse_bot::NBTaskGoal task_goal;
  std::vector<float> input{0, 0, 0, 0};
  int count = 1;
};

}  //  namespace nursebot

#endif  //  INCLUDE_NURSE_BOT_USER_INTERFACE_HPP_
