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
 * @file robot_controller.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Defining the class to control all the different joint movements of the robot
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <nurse-bot/robot_controller.hpp>


nursebot::RobotController::RobotController(
                  const nursebot::HeadControllerPtr& headCtrlPtr,
                  const nursebot::TorsoControllerPtr& torsoCtrlPtr,
                  const nursebot::ArmControllerPtr& armCtrlPtr,
                  const nursebot::GripperControllerPtr& gripperCtrlPtr)
      : headControllerPtr(headCtrlPtr),
        torsoControllerPtr(torsoCtrlPtr),
        armControllerPtr(armCtrlPtr),
        gripperControllerPtr(gripperCtrlPtr) {
}


nursebot::RobotController::~RobotController() {
}

bool nursebot::RobotController::prepare_for_detection() {
  // lower the head joint
  ROS_WARN_STREAM("RobotController()::prepare_for_detection():: "
                  << "Preparing the robot for detection.");
  return this->headControllerPtr->move_head(0.0, -0.8);
}

bool nursebot::RobotController::prepare_for_pickup() {
  ROS_WARN_STREAM("RobotController()::prepare_for_pickup():: "
                  << "Preparing the robot arm for picking obejct.");

  // default arm release position
  geometry_msgs::PoseStamped arm_safe_pose;
  arm_safe_pose.pose.position.x = 0.363;
  arm_safe_pose.pose.position.y = -0.197;
  arm_safe_pose.pose.position.z = 1.0;
  arm_safe_pose.pose.orientation.x = 0.707;
  arm_safe_pose.pose.orientation.y = 0;
  arm_safe_pose.pose.orientation.z = 0;
  arm_safe_pose.pose.orientation.w = 0.707;

  return this->move_arm(arm_safe_pose);
}

bool nursebot::RobotController::move_arm(geometry_msgs::PoseStamped goal_pose) {
  ROS_WARN_STREAM("RobotController()::move_arm():: Moving the arm.");
  return this->armControllerPtr->move_arm_ik(goal_pose);
}

bool nursebot::RobotController::close_gripper() {
  ROS_WARN_STREAM("RobotController()::close_gripper():: Closing the gripper.");
  return this->gripperControllerPtr->close_grip();
}

bool nursebot::RobotController::open_gripper() {
  ROS_WARN_STREAM("RobotController()::open_gripper():: Opening the gripper.");
  return this->gripperControllerPtr->release_grip();
}

bool nursebot::RobotController::set_to_default_pose() {
  bool status = this->headControllerPtr->set_to_default();
  status = status && this->torsoControllerPtr->set_to_default();
  status = status && this->armControllerPtr->tuck_arm();
  return status;
}
