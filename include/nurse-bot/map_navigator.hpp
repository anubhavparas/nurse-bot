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
 * @file map_navigator.hpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Declaration of the MapNavigator class
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_NURSE_BOT_MAP_NAVIGATOR_HPP_
#define INCLUDE_NURSE_BOT_MAP_NAVIGATOR_HPP_

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <memory>
#include <string>

#include <nurse-bot/pose.hpp>
#include <nurse-bot/movebaseaction_wrapper.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

namespace nursebot {

/**
 * @brief Interface for Navigation related tasks
 * 
 */
class Navigator {
 public:
  /**
   * @brief Destroy the Navigator object
   * 
   */
  virtual ~Navigator() {}

  /**
   * @brief to navigate to a particular goal position
   * 
   * @param goal_pose target goal position
   * @return true if goal position is reached successfully
   * @return false if goal postiopn is not reached
   */
  virtual bool navigate(const nursebot::Pose& goal_pose) = 0;
};


class MapNavigator : public nursebot::Navigator {
 public:
  /**
   * @brief Construct a new MapNavigator object
   * This class makes the bot move in the map frame from its current position to 
   * a specified goal position
   * 
   * @param movebase_action responsible for calling move_base action
   * 
   */
  explicit MapNavigator(
      const std::shared_ptr<nursebot::MoveBaseActionWrapper>& movebase_action);

  /**
   * @brief Destroy the MapNavigator object
   * 
   */
  virtual ~MapNavigator();

  /**
   * @brief Method to make the bot navigate to a specified goal position
   *        starting from its current position
   * 
   * @param goal_pose target position in the 'map' frame
   * @return true if goal position is reached successfully
   * @return false if goal postiopn is not reached
   */
  bool navigate(const nursebot::Pose& goal_pose) override;

 private:
  std::shared_ptr<nursebot::MoveBaseActionWrapper> movebase_action;
  std::string frame_id = "map";
};
}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_MAP_NAVIGATOR_HPP_
