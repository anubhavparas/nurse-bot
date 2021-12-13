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

#include <string>
#include <memory>
#include <nurse-bot/head_controller.hpp>

using ::testing::_;

TEST(HeadControllerTest, test_move_head) {
  ros::NodeHandle n;
  std::string head_ctrl_name;

  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  n.getParam("/head_ctrl_name", head_ctrl_name);
  ROS_WARN_STREAM("head_ctrl_name : " << head_ctrl_name);

  nursebot::HeadController head_controller(head_ctrl_name);

  bool status = head_controller.move_head(1.0, 0.0);
  EXPECT_TRUE(status);
}

TEST(HeadControllerTest, test_set_to_default) {
  ros::NodeHandle n;
  std::string head_ctrl_name;

  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  n.getParam("/head_ctrl_name", head_ctrl_name);
  ROS_WARN_STREAM("head_ctrl_name : " << head_ctrl_name);

  nursebot::HeadController head_controller(head_ctrl_name);

  bool status = head_controller.set_to_default();
  EXPECT_TRUE(status);
}
