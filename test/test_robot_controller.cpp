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
 * @file test_pick_place_controller.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Testing of PickPlaceController class
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <nurse-bot/robot_controller.hpp>

#include <nurse-bot/mock/arm_controller_mock.hpp>
#include <nurse-bot/mock/head_controller_mock.hpp>
#include <nurse-bot/mock/torso_controller_mock.hpp>
#include <nurse-bot/mock/gripper_controller_mock.hpp>



using ::testing::_;


TEST(RobotControllerTest, test_prepare_for_detection) {
  std::shared_ptr<nursebot::HeadControllerMock> head_ctrl_mock(
        new nursebot::HeadControllerMock());
  std::shared_ptr<nursebot::TorsoControllerMock> torso_ctrl_mock(
        new nursebot::TorsoControllerMock());
  std::shared_ptr<nursebot::ArmControllerMock> arm_ctrl_mock(
        new nursebot::ArmControllerMock());
  std::shared_ptr<nursebot::GripperControllerMock> gripper_ctrl_mock(
        new nursebot::GripperControllerMock());

  bool expected_status = true;
  EXPECT_CALL(*head_ctrl_mock, move_head(_, _))
            .WillRepeatedly(::testing::Return(expected_status));

  nursebot::RobotController robot_ctrl(
                              head_ctrl_mock,
                              torso_ctrl_mock,
                              arm_ctrl_mock,
                              gripper_ctrl_mock);

  bool actual_status = robot_ctrl.prepare_for_detection();

  EXPECT_EQ(actual_status, expected_status);
}

TEST(RobotControllerTest, test_prepare_for_pickup) {
  std::shared_ptr<nursebot::HeadControllerMock> head_ctrl_mock(
        new nursebot::HeadControllerMock());
  std::shared_ptr<nursebot::TorsoControllerMock> torso_ctrl_mock(
        new nursebot::TorsoControllerMock());
  std::shared_ptr<nursebot::ArmControllerMock> arm_ctrl_mock(
        new nursebot::ArmControllerMock());
  std::shared_ptr<nursebot::GripperControllerMock> gripper_ctrl_mock(
        new nursebot::GripperControllerMock());

  bool expected_status = true;
  EXPECT_CALL(*arm_ctrl_mock, move_arm_ik(_))
            .WillRepeatedly(::testing::Return(expected_status));

  nursebot::RobotController robot_ctrl(
                              head_ctrl_mock,
                              torso_ctrl_mock,
                              arm_ctrl_mock,
                              gripper_ctrl_mock);

  bool actual_status = robot_ctrl.prepare_for_pickup();

  EXPECT_EQ(actual_status, expected_status);
}

TEST(RobotControllerTest, test_close_gripper) {
  std::shared_ptr<nursebot::HeadControllerMock> head_ctrl_mock(
        new nursebot::HeadControllerMock());
  std::shared_ptr<nursebot::TorsoControllerMock> torso_ctrl_mock(
        new nursebot::TorsoControllerMock());
  std::shared_ptr<nursebot::ArmControllerMock> arm_ctrl_mock(
        new nursebot::ArmControllerMock());
  std::shared_ptr<nursebot::GripperControllerMock> gripper_ctrl_mock(
        new nursebot::GripperControllerMock());

  bool expected_status = true;
  EXPECT_CALL(*gripper_ctrl_mock, close_grip())
            .WillRepeatedly(::testing::Return(expected_status));

  nursebot::RobotController robot_ctrl(
                              head_ctrl_mock,
                              torso_ctrl_mock,
                              arm_ctrl_mock,
                              gripper_ctrl_mock);

  bool actual_status = robot_ctrl.close_gripper();

  EXPECT_EQ(actual_status, expected_status);
}

TEST(RobotControllerTest, test_open_gripper) {
  std::shared_ptr<nursebot::HeadControllerMock> head_ctrl_mock(
        new nursebot::HeadControllerMock());
  std::shared_ptr<nursebot::TorsoControllerMock> torso_ctrl_mock(
        new nursebot::TorsoControllerMock());
  std::shared_ptr<nursebot::ArmControllerMock> arm_ctrl_mock(
        new nursebot::ArmControllerMock());
  std::shared_ptr<nursebot::GripperControllerMock> gripper_ctrl_mock(
        new nursebot::GripperControllerMock());

  bool expected_status = true;
  EXPECT_CALL(*gripper_ctrl_mock, release_grip())
            .WillRepeatedly(::testing::Return(expected_status));

  nursebot::RobotController robot_ctrl(
                              head_ctrl_mock,
                              torso_ctrl_mock,
                              arm_ctrl_mock,
                              gripper_ctrl_mock);

  bool actual_status = robot_ctrl.open_gripper();

  EXPECT_EQ(actual_status, expected_status);
}


TEST(RobotControllerTest, test_set_to_default_pose) {
  std::shared_ptr<nursebot::HeadControllerMock> head_ctrl_mock(
        new nursebot::HeadControllerMock());
  std::shared_ptr<nursebot::TorsoControllerMock> torso_ctrl_mock(
        new nursebot::TorsoControllerMock());
  std::shared_ptr<nursebot::ArmControllerMock> arm_ctrl_mock(
        new nursebot::ArmControllerMock());
  std::shared_ptr<nursebot::GripperControllerMock> gripper_ctrl_mock(
        new nursebot::GripperControllerMock());

  bool expected_status = true;
  EXPECT_CALL(*head_ctrl_mock, set_to_default())
            .WillRepeatedly(::testing::Return(expected_status));
  EXPECT_CALL(*torso_ctrl_mock, set_to_default())
            .WillRepeatedly(::testing::Return(expected_status));
  EXPECT_CALL(*arm_ctrl_mock, tuck_arm())
            .WillRepeatedly(::testing::Return(expected_status));

  nursebot::RobotController robot_ctrl(
                              head_ctrl_mock,
                              torso_ctrl_mock,
                              arm_ctrl_mock,
                              gripper_ctrl_mock);

  bool actual_status = robot_ctrl.set_to_default_pose();

  EXPECT_EQ(actual_status, expected_status);
}

