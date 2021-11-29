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
 * @file movebaseaction_wrapper_mock.hpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief mock for MoveBaseActionWrapper class
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_NURSE_BOT_MOCK_MOVEBASEACTION_WRAPPER_MOCK_HPP_
#define INCLUDE_NURSE_BOT_MOCK_MOVEBASEACTION_WRAPPER_MOCK_HPP_

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>
#include <nurse-bot/movebaseaction_wrapper.hpp>
#include <nurse-bot/pose.hpp>

namespace nursebot {
class MoveBaseActionWrapperMock : public nursebot::MoveBaseActionWrapper {
 public:
  MOCK_METHOD2(sendgoal, bool(const std::string&, const nursebot::Pose&));
};
}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_MOCK_MOVEBASEACTION_WRAPPER_MOCK_HPP_