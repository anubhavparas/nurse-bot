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
 * @file gripper_controller.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Defining the class to control the gripper movement of the robot
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <nurse-bot/gripper_controller.hpp>

nursebot::GripperController::GripperController(const std::string& ctlr_name)
    : grip_control_client(std::make_shared<JointCtrlClient>(ctlr_name)) {
  ROS_WARN_STREAM("GripperController():: "
                  << "Waiting for grip controller action server to start.");
  this->grip_control_client->waitForServer();
  ROS_WARN_STREAM("GripperController():: "
                    << "grip controller action server started.");

  // Set the joint names
  grip_goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
  grip_goal.trajectory.joint_names.push_back("gripper_right_finger_joint");
  grip_goal.trajectory.points.resize(1);
  grip_goal.trajectory.points[0].positions.resize(2);
}

nursebot::GripperController::~GripperController() {
}

bool nursebot::GripperController::move_gripper() {
  // Sending goal to action server
  this->grip_control_client->sendGoal(grip_goal);
  this->grip_control_client->waitForResult();

  if (this->grip_control_client->getState()
            == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_WARN_STREAM("GripperController::move_gripper():: "
                    << "Gripper reached desired position successfully.");
  } else {
    ROS_ERROR_STREAM("GripperController::move_gripper():: "
                    << "Gripper could not reach desired position.");
    return false;
  }

  return true;
}

bool nursebot::GripperController::close_grip() {
  // Set the joint states to 0.0
  this->grip_goal.trajectory.points[0].positions[0] = 0.0;
  this->grip_goal.trajectory.points[0].positions[1] = 0.0;
  this->grip_goal.trajectory.points[0].time_from_start = ros::Duration(4.0);

  // Execute the grip action
  ROS_WARN_STREAM("GripperController::close_grip():: "
                    << "Sending goal to close the gripper.");
  return this->move_gripper();
}

bool nursebot::GripperController::release_grip() {
  // Set the joint states to 0.04
  this->grip_goal.trajectory.points[0].positions[0] = 0.04;
  this->grip_goal.trajectory.points[0].positions[1] = 0.04;
  this->grip_goal.trajectory.points[0].time_from_start = ros::Duration(4.0);

  // Execute the grip action
  ROS_WARN_STREAM("GripperController::release_grip():: "
                    << "Sending goal to open the gripper.");
  return this->move_gripper();
}
