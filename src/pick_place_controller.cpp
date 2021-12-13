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
 * @file pick_place_controller.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Defining the class to control all the pick and place tasks to be performed by the robot
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <nurse-bot/pick_place_controller.hpp>

nursebot::PickPlaceController::PickPlaceController(
    const nursebot::ArucoDetectorPtr& arucoDetectorPtr,
    const nursebot::RobotControllerPtr& robotControllerPtr)
    : arucoDetectorPtr(arucoDetectorPtr),
      robotControllerPtr(robotControllerPtr) {
}

nursebot::PickPlaceController::~PickPlaceController() {
}

bool nursebot::PickPlaceController::pick_object() {
  ROS_WARN_STREAM("PickPlaceController()::pick_object():: "
                  << "Starting to pick the object.");

  // prepare the robot for detection
  bool status = this->robotControllerPtr->prepare_for_detection();
  if (!status) {
    ROS_ERROR_STREAM("PickPlaceController()::pick_object():: "
                  << "Could not prepare the the bot for detection.");
    return status;
  }
  ROS_WARN_STREAM("PickPlaceController()::pick_object():: "
                  << "Robot prepared for object detection.");

  int count = 0;
  // start detecting the object
  while (!this->arucoDetectorPtr->is_object_detected() && count < 100) {
    ROS_WARN_STREAM("PickPlaceController()::pick_object():: "
                  << "Waiting for object to get detected.");
    ros::Duration(1).sleep();
    ros::spinOnce();
    count++;
  }

  if (!this->arucoDetectorPtr->is_object_detected()) {
    ROS_ERROR_STREAM("PickPlaceController()::pick_object():: "
                  << "Could not detect object.");
    return false;
  }
  ROS_WARN_STREAM("PickPlaceController()::pick_object():: Object Detected.");


  // move the arm to the pre-grasp position
  status = this->robotControllerPtr->prepare_for_pickup();
  // status = status && this->robotControllerPtr->move_arm(
  //                 this->arucoDetectorPtr->get_pre_grasp_pose());

  // if (!status) {
  //   ROS_ERROR_STREAM("PickPlaceController()::pick_object():: "
  //                 << "Could not reach pre-grasp position.");
  //   return status;
  // }

  // move the arm to the grasp position
  // status = status && this->robotControllerPtr->move_arm(
  //                 this->arucoDetectorPtr->get_grasp_pose());
  // if (!status) {
  //   ROS_ERROR_STREAM("PickPlaceController()::pick_object():: "
  //                 << "Could not reach object position.");
  //   return status;
  // }

  // close the grip
  status = this->robotControllerPtr->close_gripper();
  if (!status) {
    ROS_ERROR_STREAM("PickPlaceController()::pick_object():: "
                  << "Could not close the gripper.");
    return status;
  }

  // move the bot to default config pose
  status = this->robotControllerPtr->set_to_default_pose();
  if (!status) {
    ROS_ERROR_STREAM("PickPlaceController()::pick_object():: "
                  << "Could not go to default position");
    return status;
  }

  // reset the detected flag
  this->arucoDetectorPtr->object_detected(false);

  ROS_WARN_STREAM("PickPlaceController()::pick_object():: Object picked up.");
  return status;
}


bool nursebot::PickPlaceController::release_object() {
  ROS_WARN_STREAM("PickPlaceController()::release_object():: "
                  << "Starting to release the object.");

  // default arm release position
  geometry_msgs::PoseStamped arm_release_pose;
  arm_release_pose.pose.position.x = 0.5;
  arm_release_pose.pose.position.y = -0.197;
  arm_release_pose.pose.position.z = 0.9;
  arm_release_pose.pose.orientation.x = 0.707;
  arm_release_pose.pose.orientation.y = 0;
  arm_release_pose.pose.orientation.z = 0;
  arm_release_pose.pose.orientation.w = 0.707;

  // releasing the object
  bool status = this->robotControllerPtr->open_gripper();
  if (!status) {
    ROS_ERROR_STREAM("PickPlaceController()::release_object():: "
                  << "Could not open the gripper.");
    return status;
  }

  // move the bot to default config pose
  status = this->robotControllerPtr->set_to_default_pose();
  if (!status) {
    ROS_ERROR_STREAM("PickPlaceController()::release_object():: "
                  << "Could not go to default position");
    return status;
  }

  ROS_WARN_STREAM("PickPlaceController()::release_object():: Object released.");
  return status;
}
