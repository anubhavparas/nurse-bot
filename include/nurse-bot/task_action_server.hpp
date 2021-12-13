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
 * @file task_action_server.hpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Defining the class to act as action server for the task commands
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_NURSE_BOT_TASK_ACTION_SERVER_HPP_
#define INCLUDE_NURSE_BOT_TASK_ACTION_SERVER_HPP_

#include <actionlib/server/simple_action_server.h>
#include <nurse_bot/NBTaskAction.h>
#include <nurse_bot/Task.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <unordered_set>
#include <nurse-bot/task_action.hpp>

namespace nursebot {
class TaskActionServer {
 protected:
  ros::NodeHandle ros_nh;

  // NodeHandle instance must be created before this line.
  // Otherwise strange error occurs.
  actionlib::SimpleActionServer<nurse_bot::NBTaskAction> action_server;
  std::string action_name;

  // create messages that are used to published feedback/result.
  nurse_bot::NBTaskFeedback task_feedback;
  nurse_bot::NBTaskResult task_result;

 public:
  /**
   * @brief Construct a new TaskActionServer object
   * 
   * @param action_name name of the server
   * @param guidance_task_action pointer to the object to perform guidance tasks
   * @param delivery_task_action pointer to the object to perform delivery tasks
   */
  TaskActionServer(
        const std::string& action_name,
        const std::shared_ptr<nursebot::TaskAction>& guidance_task_action,
        const std::shared_ptr<nursebot::TaskAction>& delivery_task_action);

  /**
   * @brief Destroy the TaskActionServer object
   * 
   */
  virtual ~TaskActionServer();

  /**
   * @brief callback method to be executed by the server for any incoming requests
   * 
   * @param task_goal message comprising of information of the task to be performed
   */
  virtual void task_action_callback(
            const nurse_bot::NBTaskGoalConstPtr& task_goal);

 private:
  std::string move_base_server_name;
  std::shared_ptr<nursebot::TaskAction> guidance_task_action;
  std::shared_ptr<nursebot::TaskAction> delivery_task_action;
};

}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_TASK_ACTION_SERVER_HPP_
