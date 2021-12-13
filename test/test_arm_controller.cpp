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
 * @file test_arm_controller.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Testing of ArmController class
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <nurse-bot/arm_controller.hpp>

using ::testing::_;

TEST(ArmControllerTest, test_move_arm_ik) {
  // geometry_msgs::PoseStamped goal_pose;
  // goal_pose.pose.position.x = 0.363;
  // goal_pose.pose.position.y = -0.197;
  // goal_pose.pose.position.z = 1.0;
  // goal_pose.pose.orientation.x = 0.707;
  // goal_pose.pose.orientation.y = 0;
  // goal_pose.pose.orientation.z = 0;
  // goal_pose.pose.orientation.w = 0.707;

  // bool expected_status = true;

  // nursebot::ArmController arm_controller;

  // bool actual_status = arm_controller.move_arm_ik(goal_pose);

  // EXPECT_EQ(expected_status, actual_status);
}

TEST(ArmControllerTest, test_move_arm_fk) {
}

TEST(ArmControllerTest, test_tuck_arm) {
}
