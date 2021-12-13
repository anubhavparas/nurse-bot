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
 * @file head_controller.hpp
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

#ifndef INCLUDE_NURSE_BOT_HEAD_CONTROLLER_HPP_
#define INCLUDE_NURSE_BOT_HEAD_CONTROLLER_HPP_

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

namespace nursebot {
typedef control_msgs::FollowJointTrajectoryAction FollowJointTrajAction;
typedef actionlib::SimpleActionClient<FollowJointTrajAction> JointCtrlClient;

class HeadController {
 public:
  /**
   * @brief Default constructor for a new HeadController object
   * 
   */
  HeadController() {}
  /**
   * @brief Construct a new HeadController object
   * 
   */
  explicit HeadController(const std::string& ctlr_name);

  /**
   * @brief Destroy the HeadController object
   * 
   */
  virtual ~HeadController();

  /**
   * @brief Method to move the head joints of the robot
   * This will use the JointTrajectoryController action interface to control the head joints 
   * 
   * @param joint_1_postion 
   * @param joint_2_position 
   * @return true if the goal position is reached
   * @return false if the goal positions is not reached
   */
  virtual bool move_head(double joint_1_postion, double joint_2_position);

  /**
   * @brief Set the head joints to default initial position (0.0, 0.0)
   * 
   * @return true if the default goal position is reached
   * @return false if the default goal position is not reached 
   */
  virtual bool set_to_default();

 private:
  ros::NodeHandle ros_nh;
  std::shared_ptr<JointCtrlClient> head_control_client;
  control_msgs::FollowJointTrajectoryGoal head_goal;
  std::string head_ctlr_name = "/head_controller/follow_joint_trajectory";
};
}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_HEAD_CONTROLLER_HPP_
