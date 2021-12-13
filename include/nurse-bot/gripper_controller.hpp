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
 * @file gripper_controller.hpp
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

#ifndef INCLUDE_NURSE_BOT_GRIPPER_CONTROLLER_HPP_
#define INCLUDE_NURSE_BOT_GRIPPER_CONTROLLER_HPP_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace nursebot {
typedef control_msgs::FollowJointTrajectoryAction FollowJointTrajAction;
typedef actionlib::SimpleActionClient<FollowJointTrajAction> JointCtrlClient;
class GripperController {
 public:
  /**
   * @brief Default constructor for a new GripperController object
   * 
   */
  GripperController() {}

  /**
   * @brief Construct a new GripperController object
   * 
   */
  explicit GripperController(const std::string& ctlr_name);

  /**
   * @brief Destroy the GripperController object
   * 
   */
  virtual ~GripperController();

  /**
   * @brief Method to control the gripper joints to hold an object
   * This will use the JointTrajectoryController action interface to control the gripper joints
   * 
   * @return true if gripper is closed successfully
   * @return false if gripper cannot be closed successfully
   */
  virtual bool close_grip();

  /**
   * @brief Method to control the gripper joints to release an object
   * This will use the JointTrajectoryController action interface to control the gripper joints
   * 
   * @return true if gripper is opened successfully
   * @return false if gripper cannot be opened successfully
   */
  virtual bool release_grip();


 private:
  ros::NodeHandle ros_nh;
  std::shared_ptr<JointCtrlClient> grip_control_client;
  control_msgs::FollowJointTrajectoryGoal grip_goal;
  std::string grip_ctlr_name = "/gripper_controller/follow_joint_trajectory";

  /**
   * @brief Method to move the gripper joints to grip_goal postion using grip_control_client.
   * 
   * @return true if the desired gripper pose is reached.
   * @return false if the desired gripper pose is not reached.
   */
  bool move_gripper();
};
}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_GRIPPER_CONTROLLER_HPP_
