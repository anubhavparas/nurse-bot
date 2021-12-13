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
 * @file test_head_controller.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Testing of HeadController class
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <geometry_msgs/TransformStamped.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include <nurse-bot/mock/task_action_mock.hpp>
#include <nurse-bot/task_action_client.hpp>
#include <nurse-bot/task_action_server.hpp>

using ::testing::_;


TEST(TaskActionServerTest, test_task_callback) {
  ros::NodeHandle n;
  std::string task_action_server_name;

  ros::WallDuration(10.0).sleep();
  ros::spinOnce();

  n.getParam("/task_action_server_name", task_action_server_name);
  ROS_WARN_STREAM("task_action_server_name : " << task_action_server_name);

  std::unique_ptr<nursebot::TaskActionMock> task_action_mock1(
      new nursebot::TaskActionMock());

  std::unique_ptr<nursebot::TaskActionMock> task_action_mock2(
      new nursebot::TaskActionMock());

  bool expected_status = true;
  EXPECT_CALL(*task_action_mock1, perform_task(_))
            .WillOnce(::testing::Return(expected_status));

  EXPECT_CALL(*task_action_mock2, perform_task(_))
            .WillOnce(::testing::Return(expected_status));


  nursebot::TaskActionServer task_action_server(
      task_action_server_name,
      std::move(task_action_mock1),
      std::move(task_action_mock2));

  // ros::WallDuration(5.0).sleep();
  // ros::spinOnce();

  // nursebot::TaskActionClient task_ac(task_action_server_name);
  nursebot::NBActionClient task_ac(task_action_server_name, true);
  task_ac.waitForServer();

  nurse_bot::NBTaskGoal task_goal;
  task_goal.task_id = "T";
  task_goal.task_type = "G";

  task_ac.sendGoal(task_goal);
  task_ac.waitForResult();

  ROS_WARN_STREAM("Action complete.");
  auto SUCCESS = actionlib::SimpleClientGoalState::SUCCEEDED;
  bool status = task_ac.getState() == SUCCESS;

  EXPECT_TRUE(status);
}

