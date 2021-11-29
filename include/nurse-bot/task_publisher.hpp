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
 * @file task_publisher.hpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Defining the class to publish the task messages
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_NURSE_BOT_TASK_PUBLISHER_HPP_
#define INCLUDE_NURSE_BOT_TASK_PUBLISHER_HPP_

#include <nurse_bot/Task.h>
#include <ros/ros.h>
#include <string>

#include <memory>

namespace nursebot {
class TaskPublisher {
 public:
  /**
   * @brief Construct a new TaskPublisher object
   * 
   */
  TaskPublisher();

  /**
   * @brief Destroy the TaskPublisher object
   * 
   */
  virtual ~TaskPublisher();

  /**
   * @brief method to publish the messages.
   * 
   * @param task_msg task message to be published
   */
  virtual void publish(const nurse_bot::Task& task_msg);

  /**
   * @brief Get the task_msg
   * 
   * @return nurse_bot::Task
   */
  nurse_bot::Task get_task_msg() {
    return this->task_msg;
  }

 private:
  std::shared_ptr<ros::NodeHandle> ros_node_h;
  ros::Publisher task_msg_pub;
  nurse_bot::Task task_msg;
  int buffer_rate = 10;
  std::string task_topic = "/nursebot/task";
};

}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_TASK_PUBLISHER_HPP_
