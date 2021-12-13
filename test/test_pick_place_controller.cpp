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


#include <nurse-bot/mock/aruco_detector_mock.hpp>
#include <nurse-bot/mock/robot_controller_mock.hpp>
#include <nurse-bot/pick_place_controller.hpp>

using ::testing::_;


TEST(PickPlaceControllerTest, test_pick_object) {
  std::shared_ptr<nursebot::RobotControllerMock> robot_ctrl_mock(
        new nursebot::RobotControllerMock());
  std::shared_ptr<nursebot::ArucoDetectorMock> aruco_detector_mock(
        new nursebot::ArucoDetectorMock());

  bool expected_status = true;
  EXPECT_CALL(*robot_ctrl_mock, prepare_for_detection())
            .WillRepeatedly(::testing::Return(expected_status));
  EXPECT_CALL(*robot_ctrl_mock, prepare_for_pickup())
            .WillRepeatedly(::testing::Return(expected_status));
  EXPECT_CALL(*robot_ctrl_mock, close_gripper())
            .WillRepeatedly(::testing::Return(expected_status));
  EXPECT_CALL(*robot_ctrl_mock, open_gripper())
            .WillRepeatedly(::testing::Return(expected_status));
  EXPECT_CALL(*robot_ctrl_mock, set_to_default_pose())
            .WillRepeatedly(::testing::Return(expected_status));
  EXPECT_CALL(*robot_ctrl_mock, move_arm(_))
            .WillRepeatedly(::testing::Return(expected_status));

  geometry_msgs::PoseStamped expected_pose;
  EXPECT_CALL(*aruco_detector_mock, get_object_pose())
            .WillRepeatedly(::testing::Return(expected_pose));
  EXPECT_CALL(*aruco_detector_mock, get_pre_grasp_pose())
            .WillRepeatedly(::testing::Return(expected_pose));
  EXPECT_CALL(*aruco_detector_mock, get_grasp_pose())
            .WillRepeatedly(::testing::Return(expected_pose));
  EXPECT_CALL(*aruco_detector_mock, is_object_detected())
            .WillRepeatedly(::testing::Return(expected_status));
  EXPECT_CALL(*aruco_detector_mock, object_detected(_))
            .WillRepeatedly(::testing::Return());

  nursebot::PickPlaceController pick_place_ctrl(
                              aruco_detector_mock,
                              robot_ctrl_mock);

  bool actual_status = pick_place_ctrl.pick_object();

  EXPECT_EQ(actual_status, expected_status);
}

TEST(PickPlaceControllerTest, test_release_object) {
  std::unique_ptr<nursebot::RobotControllerMock> robot_ctrl_mock(
        new nursebot::RobotControllerMock());
  std::unique_ptr<nursebot::ArucoDetectorMock> aruco_detector_mock(
        new nursebot::ArucoDetectorMock());

  bool expected_status = true;
  EXPECT_CALL(*robot_ctrl_mock, prepare_for_detection())
            .WillRepeatedly(::testing::Return(expected_status));
  EXPECT_CALL(*robot_ctrl_mock, prepare_for_pickup())
            .WillRepeatedly(::testing::Return(expected_status));
  EXPECT_CALL(*robot_ctrl_mock, close_gripper())
            .WillRepeatedly(::testing::Return(expected_status));
  EXPECT_CALL(*robot_ctrl_mock, open_gripper())
            .WillRepeatedly(::testing::Return(expected_status));
  EXPECT_CALL(*robot_ctrl_mock, set_to_default_pose())
            .WillRepeatedly(::testing::Return(expected_status));
  EXPECT_CALL(*robot_ctrl_mock, move_arm(_))
            .WillRepeatedly(::testing::Return(expected_status));

  geometry_msgs::PoseStamped expected_pose;
  EXPECT_CALL(*aruco_detector_mock, get_object_pose())
            .WillRepeatedly(::testing::Return(expected_pose));
  EXPECT_CALL(*aruco_detector_mock, get_pre_grasp_pose())
            .WillRepeatedly(::testing::Return(expected_pose));
  EXPECT_CALL(*aruco_detector_mock, get_grasp_pose())
            .WillRepeatedly(::testing::Return(expected_pose));
  EXPECT_CALL(*aruco_detector_mock, is_object_detected())
            .WillRepeatedly(::testing::Return(expected_status));
  EXPECT_CALL(*aruco_detector_mock, object_detected(_))
            .WillRepeatedly(::testing::Return());

  nursebot::PickPlaceController pick_place_ctrl(
                              std::move(aruco_detector_mock),
                              std::move(robot_ctrl_mock));

  bool actual_status = pick_place_ctrl.release_object();

  EXPECT_EQ(actual_status, expected_status);
}
