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
 * @file test_map_navigator.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Testing of MapNavigator class
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>

#include <nurse-bot/mock/movebaseaction_wrapper_mock.hpp>
#include <nurse-bot/movebaseaction_wrapper.hpp>
#include <nurse-bot/pose.hpp>
#include <nurse-bot/map_navigator.hpp>


using ::testing::_;

TEST(MapNavigatorTest, testNavigateMethod) {
  std::unique_ptr<nursebot::MoveBaseActionWrapperMock> mb_action_mock(
        new nursebot::MoveBaseActionWrapperMock());

  bool expected_status = true;
  EXPECT_CALL(*mb_action_mock, sendgoal(_, _))
            .WillOnce(::testing::Return(expected_status));

  nursebot::Navigator* navigator =
              new nursebot::MapNavigator(std::move(mb_action_mock));

  nursebot::Pose goal_pose(0, 0, 0, 0, 0, 1);
  bool actual_status = navigator->navigate(goal_pose);

  EXPECT_EQ(actual_status, expected_status);

  delete navigator;
}
