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
 * @file task_action.hpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Definitions of MapNavigator class
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <geometry_msgs/Twist.h>
#include <nurse_bot/Task.h>

#include <nurse-bot/pose.hpp>
#include <nurse-bot/task_action.hpp>

nursebot::GuidanceTask::GuidanceTask(
    const std::shared_ptr<nursebot::Navigator>& navigator)
    : navigator(navigator) {
}

nursebot::GuidanceTask::~GuidanceTask() {
}

bool nursebot::GuidanceTask::perform_task(
    const nurse_bot::Task::ConstPtr& task_msg) {
  return true;
}


//////////////////////////////////////

nursebot::DeliveryTask::DeliveryTask(
    const std::shared_ptr<nursebot::Navigator>& navigator)
    : navigator(navigator) {
}

nursebot::DeliveryTask::~DeliveryTask() {
}

bool nursebot::DeliveryTask::perform_task(
    const nurse_bot::Task::ConstPtr& task_msg) {
  return true;
}
