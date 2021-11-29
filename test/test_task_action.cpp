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
 * @file test_task_action.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Testing of GuidanceTask and DeliveryTask class
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <nurse_bot/Task.h>

#include <memory>

#include <nurse-bot/mock/navigator_mock.hpp>
#include <nurse-bot/task_action.hpp>


using ::testing::_;


TEST(GuidanceTaskTest, testPerformTaskMethod_targetReached) {
  std::shared_ptr<nursebot::NavigatorMock> navigator_mock(
        new nursebot::NavigatorMock());

  bool expected_status = true;
  EXPECT_CALL(*navigator_mock, navigate(_))
            .WillOnce(::testing::Return(expected_status))
            .WillOnce(::testing::Return(expected_status));

  nursebot::TaskAction* task_action =
              new nursebot::GuidanceTask(navigator_mock);

  nurse_bot::TaskConstPtr task_msg;
  bool actual_status = task_action->perform_task(task_msg);

  EXPECT_EQ(actual_status, expected_status);

  delete task_action;
}

TEST(GuidanceTaskTest, testPerformTaskMethod_didNotReachEntity) {
  std::shared_ptr<nursebot::NavigatorMock> navigator_mock(
        new nursebot::NavigatorMock());

  bool expected_status = false;
  EXPECT_CALL(*navigator_mock, navigate(_))
            .WillRepeatedly(::testing::Return(expected_status));

  nursebot::TaskAction* task_action =
              new nursebot::GuidanceTask(navigator_mock);

  nurse_bot::TaskConstPtr task_msg;
  bool actual_status = task_action->perform_task(task_msg);

  EXPECT_EQ(actual_status, expected_status);

  delete task_action;
}

TEST(GuidanceTaskTest, testPerformTaskMethod_didNotReachTarget) {
  std::shared_ptr<nursebot::NavigatorMock> navigator_mock(
        new nursebot::NavigatorMock());

  bool expected_status = false;
  EXPECT_CALL(*navigator_mock, navigate(_))
            .WillOnce(::testing::Return(true))
            .WillRepeatedly(::testing::Return(expected_status));

  nursebot::TaskAction* task_action =
              new nursebot::GuidanceTask(navigator_mock);

  nurse_bot::TaskConstPtr task_msg;
  bool actual_status = task_action->perform_task(task_msg);

  EXPECT_EQ(actual_status, expected_status);

  delete task_action;
}

//////////////////////////////////////


TEST(DeliveryTaskTest, testPerformTaskMethod_targetReached) {
  std::shared_ptr<nursebot::NavigatorMock> navigator_mock(
        new nursebot::NavigatorMock());

  bool expected_status = true;
  EXPECT_CALL(*navigator_mock, navigate(_))
            .WillOnce(::testing::Return(expected_status))
            .WillOnce(::testing::Return(expected_status));

  nursebot::TaskAction* task_action =
              new nursebot::DeliveryTask(navigator_mock);

  nurse_bot::TaskConstPtr task_msg;
  bool actual_status = task_action->perform_task(task_msg);

  EXPECT_EQ(actual_status, expected_status);

  delete task_action;
}

TEST(DeliveryTaskTest, testPerformTaskMethod_didNotReachEntity) {
  std::shared_ptr<nursebot::NavigatorMock> navigator_mock(
        new nursebot::NavigatorMock());

  bool expected_status = false;
  EXPECT_CALL(*navigator_mock, navigate(_))
            .WillRepeatedly(::testing::Return(expected_status));

  nursebot::TaskAction* task_action =
              new nursebot::DeliveryTask(navigator_mock);

  nurse_bot::TaskConstPtr task_msg;
  bool actual_status = task_action->perform_task(task_msg);

  EXPECT_EQ(actual_status, expected_status);

  delete task_action;
}

TEST(DeliveryTaskTest, testPerformTaskMethod_didNotReachTarget) {
  std::shared_ptr<nursebot::NavigatorMock> navigator_mock(
        new nursebot::NavigatorMock());

  bool expected_status = false;
  EXPECT_CALL(*navigator_mock, navigate(_))
            .WillOnce(::testing::Return(true))
            .WillRepeatedly(::testing::Return(expected_status));

  nursebot::TaskAction* task_action =
              new nursebot::DeliveryTask(navigator_mock);

  nurse_bot::TaskConstPtr task_msg;
  bool actual_status = task_action->perform_task(task_msg);

  EXPECT_EQ(actual_status, expected_status);

  delete task_action;
}
