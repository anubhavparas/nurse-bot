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
 * @file arm_controller.hpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Defining the class to control the arm movement of the robot
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_NURSE_BOT_ARM_CONTROLLER_HPP_
#define INCLUDE_NURSE_BOT_ARM_CONTROLLER_HPP_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <map>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

namespace nursebot {

typedef control_msgs::FollowJointTrajectoryAction FollowJointTrajAction;
typedef actionlib::SimpleActionClient<FollowJointTrajAction> JointCtrlClient;
class ArmController {
 public:
  /**
   * @brief Construct a new ArmController object
   * 
   */
  ArmController();

  /**
   * @brief Destroy the ArmController object
   * 
   */
  virtual ~ArmController();

  /**
   * @brief Method to move the arm to a particular position in cartesian system.
   * This will use the MoveIt! interface to plan and solve for IK and reach the specific location
   * 
   * @param goal_pose desired goal position of the end effector link
   * @return true if reached the desired goal pose successfully
   * @return false if the arm cannot reach the specified desired goal successfully
   */
  virtual bool move_arm_ik(geometry_msgs::PoseStamped goal_pose);

  /**
   * @brief Method to move the arm to a particular position given the joint positions.
   * This will use the MoveIt! interface to plan and solve for FK and reach the specific location
   * 
   * @param joint_angles list of joint positions of the torso and the arm
   *                      index 0 contains the position of the torso_lift_joint
   *                      index 1-7 containts the positions of the respective arm joints 1-7
   * @return true if reached the desired goal pose successfully
   * @return false if the arm cannot reach the specified desired goal successfully
   */
  virtual bool move_arm_fk(const std::vector<double>& joint_angles);

  /**
   * @brief Method to tuck the arm in a safe position
   * 
   * @return true if the arm is tucked in the safe position
   * @return false if the arm is not tucked in a safe position
   */
  virtual bool tuck_arm();

 private:
  ros::NodeHandle ros_nh;
  std::unordered_map<std::string, double> target_position;
  bool is_arm_tucked = false;

  /**
   * @brief Method to plan the motion and move the group_arm_torso joint group
   * 
   * @return true if the planning and motion is successful
   * @return false if the planning and motion is not successful
   */
  bool plan_and_move_arm(
    moveit::planning_interface::MoveGroupInterface& group_arm_torso);  // NOLINT-CPP
};

}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_ARM_CONTROLLER_HPP_
