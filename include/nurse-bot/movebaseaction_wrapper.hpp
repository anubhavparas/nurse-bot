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
 * @file movebaseaction_wrapper.hpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Wrapper around the ROS MoveBaseAction
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_NURSE_BOT_MOVEBASEACTION_WRAPPER_HPP_
#define INCLUDE_NURSE_BOT_MOVEBASEACTION_WRAPPER_HPP_

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <nurse-bot/pose.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

namespace nursebot {
class MoveBaseActionWrapper {
 public:
  MoveBaseActionWrapper() {}
  /**
   * @brief Construct a new MoveBaseActionWrapper object
   *        and sets up the necessary ros topics for the ActionInterface
   * @param name The action name. Defines the namespace in which the action communicates
   * @param spin_thread If true, spins up a thread to service this action's subscriptions. 
   *                    If false, then the user has to call ros::spin() themselves. 
   *                    Defaults to True
   */
  explicit MoveBaseActionWrapper(const std::string& name,
                                 bool spin_thread = true);

  /**
   * @brief Destroy the MoveBaseActionWrapper object
   * 
   */
  virtual ~MoveBaseActionWrapper();

  /**
   * @brief Sends a goal to the ActionServer, and also registers callbacks
   * 
   * @param frame_id name of the frame wrt which motion will be performed
   * @param goal goal to send to move_base using the move_base_msgs::MoveBaseGoal
   * @return true if goal reached successfully
   * @return false if goal is not reached successfully
   */
  virtual bool sendgoal(const std::string& frame_id,
                        const nursebot::Pose& goal);

 private:
  std::shared_ptr<MoveBaseClient> mb_actionclient;

  /**
   * @brief method to map nursebot goal object to movebase goal object
   * 
   * @param nursebot_goal 
   * @param move_base_goal 
   */
  void nursebot_goal_to_movebase_goal(
                const nursebot::Pose& nursebot_goal,
                move_base_msgs::MoveBaseGoal& move_base_goal);  // NOLINT-CPP
};
}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_MOVEBASEACTION_WRAPPER_HPP_
