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
 * @file torso_controller.hpp
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

#ifndef INCLUDE_NURSE_BOT_TORSO_CONTROLLER_HPP_
#define INCLUDE_NURSE_BOT_TORSO_CONTROLLER_HPP_

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace nursebot {
typedef control_msgs::FollowJointTrajectoryAction FollowJointTrajAction;
typedef actionlib::SimpleActionClient<FollowJointTrajAction> JointCtrlClient;

class TorsoController {
 public:
  /**
   * @brief Default constructor for a new TorsoController object
   * 
   */
  TorsoController() {}

  /**
     * @brief Construct a new TorsoController object
     * 
     */
  explicit TorsoController(const std::string& ctlr_name);

  /**
     * @brief Destroy the TorsoController object
     * 
     */
  virtual ~TorsoController();

  /**
   * @brief Method to move the torso joint of the robot
   * This will use the JointTrajectoryController action interface to control the torso joint 
   * 
   * @param torso_goal_position 
   * @return true if the goal position is reached
   * @return false if the goal positions is not reached
   */
  virtual bool move_torso(double torso_goal_position);

  /**
   * @brief Set the torso joint to default initial position (0.15)
   * 
   * @return true if the default goal position is reached
   * @return false if the default goal position is not reached 
   */
  virtual bool set_to_default();

 private:
  ros::NodeHandle ros_nh;
  std::shared_ptr<JointCtrlClient> torso_control_client;
  control_msgs::FollowJointTrajectoryGoal torso_goal;
  std::string torso_ctlr_name = "/torso_controller/follow_joint_trajectory";
};

}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_TORSO_CONTROLLER_HPP_
