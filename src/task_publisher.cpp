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
 * @file task_publisher.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Definitions of TaskPublisher class
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <nurse_bot/Task.h>

#include <nurse-bot/task_publisher.hpp>

nursebot::TaskPublisher::TaskPublisher()
    : ros_node_h(std::make_shared<ros::NodeHandle>("~")) {
  this->task_msg_pub = this->ros_node_h->advertise<nurse_bot::Task>(
      this->task_topic, buffer_rate);
}

nursebot::TaskPublisher::~TaskPublisher() {
}

void nursebot::TaskPublisher::publish(const nurse_bot::Task& task_msg) {
  ROS_WARN_STREAM("TaskPublisher:: Publishing a message");
  ros::Rate loop_rate(1);
  this->task_msg = task_msg;
  while (ros::ok()) {
    this->task_msg_pub.publish(task_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
