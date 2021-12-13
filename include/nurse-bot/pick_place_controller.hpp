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
 * @file pick_place_controller.hpp
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

#ifndef INCLUDE_NURSE_BOT_PICK_PLACE_CONTROLLER_HPP_
#define INCLUDE_NURSE_BOT_PICK_PLACE_CONTROLLER_HPP_

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <nurse-bot/aruco_detector.hpp>
#include <nurse-bot/robot_controller.hpp>


namespace nursebot {
typedef std::shared_ptr<nursebot::ArucoDetector> ArucoDetectorPtr;
typedef std::shared_ptr<nursebot::RobotController> RobotControllerPtr;

class PickPlaceController {
 public:
  /**
   * @brief Default constructor a new PickPlaceController object
   * 
   */
  PickPlaceController() {}
  /**
   * @brief Construct a new PickPlaceController object
   * 
   * @param arucoDetectorPtr reference to detect the aruco tags on the object
   * @param robotControllerPtr refercence to control all the robot movements
   */
  PickPlaceController(const ArucoDetectorPtr& arucoDetectorPtr,
                      const RobotControllerPtr& robotControllerPtr);

  /**
   * @brief Destroy the PickPlaceController object
   * 
   */
  virtual ~PickPlaceController();

  /**
   * @brief Method to detect and pick the object once it is near the object.
   * 
   * @return true if the object is successfully detected and picked up.
   * @return false if the object is not detected or picked up successfully.
   */
  virtual bool pick_object();

  /**
   * @brief Method to release the object once it is near the dropping location.
   * 
   * @return true if the object is successfully dropped/released.
   * @return false if the object is not dropped/released successfully.
   */
  virtual bool release_object();

 private:
  ArucoDetectorPtr arucoDetectorPtr;
  RobotControllerPtr robotControllerPtr;
};
}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_PICK_PLACE_CONTROLLER_HPP_
