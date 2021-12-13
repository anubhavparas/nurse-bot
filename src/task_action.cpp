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
  geometry_msgs::PoseStamped entity_position = task_msg->entity_position;
  geometry_msgs::PoseStamped target_position = task_msg->target_position;

  Pose entity_loc_pose(entity_position.pose.position.x,
                      entity_position.pose.position.y,
                      0,
                      0, 0, 0, 1);
  ROS_WARN_STREAM("GuidanceTask::perform_task():: "
                  << "Sending the bot to the object.");
  bool status = this->navigator->navigate(entity_loc_pose);

  if (!status) {
    ROS_ERROR_STREAM("GuidanceTask::perform_task():: "
                      << "Task to move towards the object failed!");
    return false;
  }

  ROS_WARN_STREAM("GuidanceTask::perform_task():: "
                  << "Bot reached at the requested location.");

  Pose target_loc_pose(target_position.pose.position.x,
                       target_position.pose.position.y,
                       0,
                       0, 0, 0, 1);
  ROS_WARN_STREAM("GuidanceTask::perform_task():: "
                  << "Started guiding the person towards the target position");
  status = this->navigator->navigate(target_loc_pose);

  if (!status) {
    ROS_ERROR_STREAM("GuidanceTask::perform_task():: "
                      << "Task to guide the person failed!");
    return false;
  }

  return true;
}


//////////////////////////////////////

nursebot::DeliveryTask::DeliveryTask(
    const std::shared_ptr<nursebot::Navigator>& navigator,
    const std::shared_ptr<nursebot::PickPlaceController>& pick_place_ctlr)
    : navigator(navigator),
      pick_place_ctlr(pick_place_ctlr) {
}


nursebot::DeliveryTask::~DeliveryTask() {
}



bool nursebot::DeliveryTask::perform_task(
    const nurse_bot::Task::ConstPtr& task_msg) {
  geometry_msgs::PoseStamped entity_position = task_msg->entity_position;
  geometry_msgs::PoseStamped target_position = task_msg->target_position;

  // Pose entity_loc_pose(1.3, -2.7, 0, 0, 0, 0, 1);  // hard coded
  Pose entity_loc_pose(entity_position.pose.position.x,
                       entity_position.pose.position.y,
                       0,
                       0, 0, 1, 0);
  ROS_WARN_STREAM("DeliveryTask::perform_task():: "
                  << "Sending the bot to the person.");
  bool status = this->navigator->navigate(entity_loc_pose);

  if (!status) {
    ROS_ERROR_STREAM("DeliveryTask::perform_task():: "
                      << "Task to move towards the person failed!");
    return false;
  }

  ROS_WARN_STREAM("DeliveryTask::perform_task():: "
                  << "Bot reached at the requested location.");

  // picking the object
  status = this->pick_place_ctlr->pick_object();
  if (!status) {
    ROS_ERROR_STREAM("DeliveryTask::perform_task():: "
                      << "Task to pick the object failed!");
    return false;
  }

  // Pose target_loc_pose(-1, 8, 0, 0, 0, 0, 1);  // hard coded
  Pose target_loc_pose(target_position.pose.position.x,
                       target_position.pose.position.y,
                       0,
                       0, 0, 0, 1);
  ROS_WARN_STREAM("DeliveryTask::perform_task():: "
                  << "Started delivering the object to the target position");
  status = this->navigator->navigate(target_loc_pose);
  if (!status) {
    ROS_ERROR_STREAM("DeliveryTask::perform_task():: "
                      << "Task to reach the target location failed!");
    return false;
  }

  // release the object
  status = this->pick_place_ctlr->release_object();
  if (!status) {
    ROS_ERROR_STREAM("DeliveryTask::perform_task():: "
                      << "Task to deliver the object failed!");
    return false;
  }

  return true;
}
