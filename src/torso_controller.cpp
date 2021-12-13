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
 * @file torso_controller.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Defining the class to control the torso movement of the robot
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <nurse-bot/torso_controller.hpp>


nursebot::TorsoController::TorsoController(const std::string& ctlr_name)
    : torso_control_client(std::make_shared<JointCtrlClient>(ctlr_name)) {
  ROS_WARN_STREAM("TorsoController()::"
                  << "Waiting for torso controller action server to start.");
  this->torso_control_client->waitForServer();
  ROS_WARN_STREAM("TorsoController():: "
                    << "torso controller action server started.");

  // setting the values for torso goal
  this->torso_goal.trajectory.joint_names.push_back("torso_lift_joint");
  this->torso_goal.trajectory.points.resize(1);
  this->torso_goal.trajectory.points[0].positions.resize(1);
}

nursebot::TorsoController::~TorsoController() {
}

bool nursebot::TorsoController::move_torso(double torso_goal_position) {
  // Set the joint state to torso_goal_position
  this->torso_goal.trajectory.points[0].positions[0] = torso_goal_position;
  this->torso_goal.trajectory.points[0].time_from_start = ros::Duration(4.0);

  // Execute the torso action
  ROS_WARN_STREAM("TorsoController::move_torso():: "
                    << "Sending goal to move torso.");

  // Sending goal to action server
  this->torso_control_client->sendGoal(this->torso_goal);
  this->torso_control_client->waitForResult();

  if (this->torso_control_client->getState()
            == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_WARN_STREAM("TorsoController::move_torso():: "
                    << "Torso reached desired position successfully.");
  } else {
    ROS_ERROR_STREAM("TorsoController::move_torso():: "
                    << "Torso could not reach desired position.");
    return false;
  }

  return true;
}

bool nursebot::TorsoController::set_to_default() {
  ROS_WARN_STREAM("TorsoController::set_to_default():: "
                    << "Setting torso to default position.");

  return this->move_torso(0.15);
}
