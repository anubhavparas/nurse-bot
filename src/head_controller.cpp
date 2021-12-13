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
 * @file head_controller.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Defining the class to control the head movement of the robot
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <nurse-bot/head_controller.hpp>

nursebot::HeadController::HeadController(const std::string& ctlr_name)
    : head_control_client(std::make_shared<JointCtrlClient>(ctlr_name)) {
  ROS_WARN_STREAM("HeadController()::"
                  << "Waiting for head controller action server to start.");
  this->head_control_client->waitForServer();
  ROS_WARN_STREAM("HeadController():: "
                    << "head controller action server started.");

  // setting the values for head goal
  this->head_goal.trajectory.joint_names.push_back("head_1_joint");
  this->head_goal.trajectory.joint_names.push_back("head_2_joint");
  this->head_goal.trajectory.points.resize(1);
  this->head_goal.trajectory.points[0].positions.resize(2);
}

nursebot::HeadController::~HeadController() {
}

bool nursebot::HeadController::move_head(
                double joint_1_position,
                double joint_2_position) {
  // Set the joint state to head_goal_position
  this->head_goal.trajectory.points[0].positions[0] = joint_1_position;
  this->head_goal.trajectory.points[0].positions[1] = joint_2_position;
  this->head_goal.trajectory.points[0].time_from_start = ros::Duration(4.0);

  // Execute the head action
  ROS_WARN_STREAM("HeadController::move_head():: "
                    << "Sending goal to move head.");

  // Sending goal to action server
  this->head_control_client->sendGoal(this->head_goal);
  this->head_control_client->waitForResult();

  if (this->head_control_client->getState()
            == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_WARN_STREAM("HeadController::move_head():: "
                    << "head reached desired position successfully.");
  } else {
    ROS_ERROR_STREAM("HeadController::move_head():: "
                    << "head could not reach desired position.");
    return false;
  }

  return true;
}

bool nursebot::HeadController::set_to_default() {
  ROS_WARN_STREAM("HeadController::set_to_default():: "
                    << "Setting head to default position.");

  return this->move_head(0.0, 0.0);
}
