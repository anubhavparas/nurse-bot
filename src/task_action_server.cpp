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
 * @file task_action_server.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Definitions of TaskActionServer class
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <nurse-bot/task_action_server.hpp>
#include <nurse-bot/map_navigator.hpp>
#include <nurse-bot/movebaseaction_wrapper.hpp>

nursebot::TaskActionServer::TaskActionServer(
          const std::string& action_name,
          const std::shared_ptr<nursebot::TaskAction>& guidance_task_action,
          const std::shared_ptr<nursebot::TaskAction>& delivery_task_action) :
      action_name(action_name),
      guidance_task_action(guidance_task_action),
      delivery_task_action(delivery_task_action),
      action_server(
        ros_nh,
        action_name,
        boost::bind(&TaskActionServer::task_action_callback, this, _1),
        false) {
  ROS_WARN_STREAM("TaskActionServer():: Starting server...");
  this->action_server.start();
  ROS_WARN_STREAM("TaskActionServer():: Server started...");
}

nursebot::TaskActionServer::~TaskActionServer() {
  ROS_WARN_STREAM("TaskActionServer():: Destroying TaskActionServer");
}


void nursebot::TaskActionServer::task_action_callback(
    const nurse_bot::NBTaskGoalConstPtr& task_goal) {

  ROS_WARN_STREAM("TaskActionServer:: Received request");

  nurse_bot::Task task_msg;
  task_msg.task_id = task_goal->task_id;
  task_msg.task_type = task_goal->task_type;
  task_msg.entity_position = task_goal->entity_position;
  task_msg.target_position = task_goal->target_position;

  nurse_bot::TaskConstPtr msg_ptr(&task_msg);
  ROS_WARN_STREAM("TaskActionServer:: Sending the task_msg to perform_task.");
  bool task_status = true;

  if (task_msg.task_type == "Guidance" || task_msg.task_type == "G") {
    task_status = this->guidance_task_action->perform_task(msg_ptr);
  }

  if (task_msg.task_type == "Delivery" || task_msg.task_type == "D") {
    task_status = this->delivery_task_action->perform_task(msg_ptr);
  }

  if (task_status) {
    ROS_WARN_STREAM("TaskActionServer:: Task completed successfully"
                    << " | Task ID: " << task_goal->task_id
                    << " | Task Type: " << task_goal->task_type);

    this->task_result.status = "SUCCESS";
    this->action_server.setSucceeded(this->task_result);
  }
  ROS_WARN_STREAM("TaskActionServer:: Exiting Callback method");
}

