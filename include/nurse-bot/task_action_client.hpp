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
 * @file task_action_client.hpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Defining the class to act as action client for the TaskActionServer
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_NURSE_BOT_TASK_ACTION_CLIENT_HPP_
#define INCLUDE_NURSE_BOT_TASK_ACTION_CLIENT_HPP_

#include <actionlib/client/simple_action_client.h>
#include <nurse_bot/NBTaskAction.h>
#include <nurse_bot/Task.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <nurse-bot/task_action.hpp>


namespace nursebot {

typedef actionlib::SimpleActionClient<nurse_bot::NBTaskAction> NBActionClient;

class TaskActionClient {
 public:
  /**
   * @brief Construct a new Task Action Client object
   * 
   * @param action_server_name
   */
  explicit TaskActionClient(const std::string& action_server_name);

  /**
   * @brief Destroy the TaskActionClient object
   * 
   */
  virtual ~TaskActionClient();

  /**
   * @brief method to send the action/task request to the action server
   * 
   * @param task_goal request payload
   * @return true 
   * @return false 
   */
  virtual bool request_action(const nurse_bot::NBTaskGoal& task_goal);

  /**
   * @brief method to wait for the action to complete
   * 
   */
  virtual void waitForResult();

  /**
   * @brief Get the state of the action request
   * 
   * @return SimpleClientGoalState::StateEnum 
   */
//   virtual SimpleClientGoalState getState();

 private:
  std::shared_ptr<nursebot::NBActionClient> action_client;
  void done_callback(const actionlib::SimpleClientGoalState& state,
                     const nurse_bot::NBTaskResultConstPtr& result);
};

}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_TASK_ACTION_CLIENT_HPP_
