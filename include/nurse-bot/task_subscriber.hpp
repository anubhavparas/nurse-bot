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
 * @file task_subscriber.hpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Defining the class to subscribe to the task messages
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_NURSE_BOT_TASK_SUBSCRIBER_HPP_
#define INCLUDE_NURSE_BOT_TASK_SUBSCRIBER_HPP_

#include <nurse_bot/Task.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <unordered_set>
#include <nurse-bot/task_action.hpp>

namespace nursebot {
class TaskSubscriber {
 public:
  /**
   * @brief Construct a new TaskSubscriber object
   * 
   */
  TaskSubscriber();

  /**
   * @brief Destroy the TaskSubscriber object
   * 
   */
  virtual ~TaskSubscriber();

  /**
   * @brief Get the task_msg_ptr
   * 
   * @return nurse_bot::TaskConstPtr 
   */
  nurse_bot::TaskConstPtr get_task_msg_ptr() {
    return this->task_msg_ptr;
  }


 private:
  std::shared_ptr<ros::NodeHandle> ros_node_h;
  std::shared_ptr<nursebot::TaskAction> task_action;

  ros::Subscriber task_msg_sub;
  int buffer_rate = 10;
  nurse_bot::TaskConstPtr task_msg_ptr;
  std::string task_topic = "/nursebot/task";

  std::unordered_set<std::string> task_ids;


  /**
   * @brief callback message for the subscriber
   * 
   * @param task_msg task message read from the topic
   */
  void task_callback(const nurse_bot::Task::ConstPtr& task_msg);
};

}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_TASK_SUBSCRIBER_HPP_
