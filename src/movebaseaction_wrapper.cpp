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
 * @file movebaseaction_wrapper.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Definitions of MoveBaseActionWrapper class
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <nurse-bot/movebaseaction_wrapper.hpp>

#include <memory>

nursebot::MoveBaseActionWrapper::MoveBaseActionWrapper(
                                const std::string & name,
                                bool spin_thread)
    : mb_actionclient(std::make_shared<MoveBaseClient>(name, spin_thread)) {
  // wait for the action server to come up
  while (!this->mb_actionclient->waitForServer(ros::Duration(5.0))) {
    ROS_WARN_STREAM("Waiting for the move_base action server to come up");
  }
}

nursebot::MoveBaseActionWrapper::~MoveBaseActionWrapper() {
}

void nursebot::MoveBaseActionWrapper::nursebot_goal_to_movebase_goal(
                const nursebot::Pose& nursebot_goal,
                move_base_msgs::MoveBaseGoal& move_base_goal) {
  move_base_goal.target_pose.pose.position.x = nursebot_goal.x;
  move_base_goal.target_pose.pose.position.y = nursebot_goal.y;
  move_base_goal.target_pose.pose.position.z = nursebot_goal.z;

  move_base_goal.target_pose.pose.orientation.x = nursebot_goal.qx;
  move_base_goal.target_pose.pose.orientation.y = nursebot_goal.qy;
  move_base_goal.target_pose.pose.orientation.z = nursebot_goal.qz;
  move_base_goal.target_pose.pose.orientation.w = nursebot_goal.qw;
}

bool nursebot::MoveBaseActionWrapper::sendgoal(const std::string& frame_id,
                                               const nursebot::Pose& goal) {
  move_base_msgs::MoveBaseGoal move_base_goal;
  move_base_goal.target_pose.header.frame_id = frame_id;
  move_base_goal.target_pose.header.stamp = ros::Time::now();

  this->nursebot_goal_to_movebase_goal(goal, move_base_goal);

  ROS_WARN_STREAM("Sending goal..");
  this->mb_actionclient->sendGoal(move_base_goal);

  this->mb_actionclient->waitForResult();

  auto SUCCESS = actionlib::SimpleClientGoalState::SUCCEEDED;
  if (mb_actionclient->getState() == SUCCESS) {
    ROS_WARN_STREAM("MoveBaseActionWrapper::sendgoal():: "
                    << "Robot has reached the specified goal");
    return true;
  } else {
      ROS_ERROR_STREAM("MoveBaseActionWrapper::sendgoal():: "
                     << "Base failed to move and reach the specified goal.");
      return false;
  }
}

