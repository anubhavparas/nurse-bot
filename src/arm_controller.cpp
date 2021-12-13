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
 * @file arm_controller.cpp
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

#include <nurse-bot/arm_controller.hpp>

nursebot::ArmController::ArmController() {
}

nursebot::ArmController::~ArmController() {
}

bool nursebot::ArmController::move_arm_ik(
    geometry_msgs::PoseStamped goal_pose) {

  // ros::AsyncSpinner spinner(1);
  // spinner.start();
  goal_pose.header.frame_id = "base_footprint";

  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  // choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");

  // Set the frame to plan in
  group_arm_torso.setPoseReferenceFrame("base_footprint");

  // Set the goal pose
  group_arm_torso.setPoseTarget(goal_pose);

  ROS_WARN_STREAM("ArmController::move_arm_ik():: "
                  << "Planning to move "
                  << group_arm_torso.getEndEffectorLink()
                  << " to a target pose expressed in "
                  << group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  bool status = this->plan_and_move_arm(group_arm_torso);
  // spinner.stop();
  return status;
}

bool nursebot::ArmController::move_arm_fk(
    const std::vector<double>& joint_angles) {

  // setting the target joints angles for the torso and the arm
  target_position["torso_lift_joint"] = joint_angles[0];
  target_position["arm_1_joint"] = joint_angles[1];
  target_position["arm_2_joint"] = joint_angles[2];
  target_position["arm_3_joint"] = joint_angles[3];
  target_position["arm_4_joint"] = joint_angles[4];
  target_position["arm_5_joint"] = joint_angles[5];
  target_position["arm_6_joint"] = joint_angles[6];
  target_position["arm_7_joint"] = joint_angles[7];

  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  std::vector<std::string> torso_arm_joint_names;

  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  // choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");

  torso_arm_joint_names = group_arm_torso.getJoints();

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  for (auto& joint_name : torso_arm_joint_names) {
    if (target_position.count(joint_name) > 0) {
      ROS_WARN_STREAM("ArmController::move_arm_fk():: "
                      << joint_name
                      << " goal position: "
                      << target_position[joint_name]);

      group_arm_torso.setJointValueTarget(
              joint_name,
              target_position[joint_name]);
    }
  }

  bool status = this->plan_and_move_arm(group_arm_torso);
  // spinner.stop();
  return status;
}


bool nursebot::ArmController::plan_and_move_arm(
      moveit::planning_interface::MoveGroupInterface& group_arm_torso) {
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Set the maximum time to plan
  group_arm_torso.setPlanningTime(10.0);
  bool success = group_arm_torso.plan(my_plan)
                      == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  if (!success) {
    ROS_ERROR_STREAM("ArmController::plan_and_move_arm():: No plan found.");
    return false;
  }

  ROS_WARN_STREAM("ArmController::plan_and_move_arm():: "
                  << "Plan found in "
                  << my_plan.planning_time_
                  << " seconds.");

  ros::Time start = ros::Time::now();

  // Execute the plan
  group_arm_torso.move();

  ROS_INFO_STREAM("ArmController::plan_and_move_arm():: "
                  << "Motion duration: "
                  << (ros::Time::now() - start).toSec());

  return true;
}

bool nursebot::ArmController::tuck_arm() {
  std::vector<double> arm_torso_joints_angles;
  // torso_lift_joint value
  arm_torso_joints_angles.push_back(0.09);

  // arm_joint values: joints 1-7
  arm_torso_joints_angles.push_back(0.20);
  arm_torso_joints_angles.push_back(-1.34);
  arm_torso_joints_angles.push_back(-0.20);
  arm_torso_joints_angles.push_back(1.94);
  arm_torso_joints_angles.push_back(-1.57);
  arm_torso_joints_angles.push_back(1.37);
  arm_torso_joints_angles.push_back(0.0);

  ROS_WARN_STREAM("ArmController::tuck_arm():: "
                  << "Starting to tuck the arm..");
  bool status = this->move_arm_fk(arm_torso_joints_angles);

  this->is_arm_tucked = status;
  return status;
}
