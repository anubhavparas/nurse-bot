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
 * @file robot_controller.hpp
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

#ifndef INCLUDE_NURSE_BOT_ROBOT_CONTROLLER_HPP_
#define INCLUDE_NURSE_BOT_ROBOT_CONTROLLER_HPP_

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

#include <nurse-bot/arm_controller.hpp>
#include <nurse-bot/gripper_controller.hpp>
#include <nurse-bot/head_controller.hpp>
#include <nurse-bot/torso_controller.hpp>

namespace nursebot {
typedef std::shared_ptr<nursebot::HeadController> HeadControllerPtr;
typedef std::shared_ptr<nursebot::TorsoController> TorsoControllerPtr;
typedef std::shared_ptr<nursebot::ArmController> ArmControllerPtr;
typedef std::shared_ptr<nursebot::GripperController> GripperControllerPtr;

class RobotController {
 public:
  /**
   * @brief Construct a new RobotController object
   * 
   */
  RobotController() {}
  /**
   * @brief Construct a new RobotController object
   * 
   * @param headCtrlPtr reference to control head movements
   * @param torsoCtrlPtr reference to control torso movements
   * @param armCtrlPtr reference to control arm movements
   * @param gripperCtrlPtr reference to control gripper movements
   */
  RobotController(const HeadControllerPtr& headCtrlPtr,
                  const TorsoControllerPtr& torsoCtrlPtr,
                  const ArmControllerPtr& armCtrlPtr,
                  const GripperControllerPtr& gripperCtrlPtr);

  /**
   * @brief Destroy the RobotController object
   * 
   */
  virtual ~RobotController();

  /**
   * @brief Makes the robot reach a joint configuration so that it can detect objects
   * 
   * @return true if the configuration was reached successfully
   * @return false if the configuration was not reached successfully
   */
  virtual bool prepare_for_detection();

  /**
   * @brief Makes the robot reach a joint configuration so that it can plan the IK properly
   * 
   * @return true if the configuration was reached successfully
   * @return false if the configuration was not reached successfully
   */
  virtual bool prepare_for_pickup();

  /**
   * @brief Move the arm to the specified desired position in cartesian system
   * 
   * @param goal_pose 
   * @return true if reached the desired goal pose successfully
   * @return false if the arm cannot reach the specified desired goal successfully 
   */
  virtual bool move_arm(geometry_msgs::PoseStamped goal_pose);

  /**
   * @brief Method to close the gripper
   * 
   * @return true if gripper closed successfully
   * @return false if gripper is not closed successfully
   */
  virtual bool close_gripper();

  /**
   * @brief Method to open the gripper
   * 
   * @return true if gripper open successfully
   * @return false if gripper is not opened successfully
   */
  virtual bool open_gripper();

  /**
   * @brief Set the robot joints to default position
   * 
   * @return true if the default goal position is reached
   * @return false if the default goal position is not reached 
   */
  virtual bool set_to_default_pose();

 private:
  HeadControllerPtr headControllerPtr;
  TorsoControllerPtr torsoControllerPtr;
  ArmControllerPtr armControllerPtr;
  GripperControllerPtr gripperControllerPtr;
};
}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_ROBOT_CONTROLLER_HPP_
