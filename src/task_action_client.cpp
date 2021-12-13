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
 * @file task_action_client.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Definitions of TaskActionClient class
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <nurse-bot/task_action_client.hpp>

nursebot::TaskActionClient::TaskActionClient(
      const std::string& action_server_name)
      :
      action_client(std::make_shared<NBActionClient>(
                                     action_server_name, true)) {
  ROS_WARN_STREAM("TaskActionClient():: Waiting for the action server ("
            << action_server_name
            << ") to start");
  this->action_client->waitForServer();
  ROS_WARN_STREAM("TaskActionClient():: TaskActionServer started");
}

nursebot::TaskActionClient::~TaskActionClient() {
}

void nursebot::TaskActionClient::waitForResult() {
  this->action_client->waitForResult();
}

actionlib::SimpleClientGoalState nursebot::TaskActionClient::getState() {
  return this->action_client->getState();
}



bool nursebot::TaskActionClient::request_action(
        const nurse_bot::NBTaskGoal& task_goal) {
  ROS_WARN_STREAM("TaskActionClient::request_action():: Sending request.. ");
  this->action_client->sendGoal(task_goal);
}

// void nursebot::TaskActionClient::done_callback(
//                      const actionlib::SimpleClientGoalState& state,
//                      const nurse_bot::NBTaskResultConstPtr& result) {
//   // ROS_WARN_STREAM("TaskActionClient::done_callback():: Task finished "
//   //                 << state.toString().c_str());

//   // ROS_WARN_STREAM("TaskActionClient::done_callback():: "
//   //                 << result->status);
// }

