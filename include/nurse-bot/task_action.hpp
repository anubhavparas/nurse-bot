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
 * @file task_action.hpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Declaration of the TaskAction interface and its derived classes
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_NURSE_BOT_TASK_ACTION_HPP_
#define INCLUDE_NURSE_BOT_TASK_ACTION_HPP_

#include <nurse_bot/Task.h>
#include <geometry_msgs/PoseStamped.h>

#include <memory>
#include <nurse-bot/map_navigator.hpp>
#include <nurse-bot/pick_place_controller.hpp>

namespace nursebot {

/**
 * @brief Interface for the task actions nursebot can perform
 * 
 */
class TaskAction {
 public:
  virtual ~TaskAction() {}
  virtual bool perform_task(const nurse_bot::Task::ConstPtr& task_msg) = 0;
};

class GuidanceTask : public nursebot::TaskAction {
 public:
  /**
   * @brief Construct a new GuidanceTask object
   * 
   * @param navigator object responsible for robot motion
   */
  explicit GuidanceTask(const std::shared_ptr<nursebot::Navigator>& navigator);

  /**
   * @brief Destroy the GuidanceTask object
   * 
   */
  virtual ~GuidanceTask();

  /**
   * @brief method to contain the logic to perform the guidance task
   * 
   * @param task_msg message object containing information about the task
   */
  bool perform_task(
      const nurse_bot::Task::ConstPtr& task_msg) override;

 private:
  /**
   * @brief pointer to navigator object responsible for moving the bot or its parts
   * 
   */
  std::shared_ptr<nursebot::Navigator> navigator;
};

class DeliveryTask : public nursebot::TaskAction {
 public:
  /**
   * @brief Construct a new DeliveryTask object
   * 
   * @param navigator object responsible for robot motion
   * @param pick_place_ctlr object responsible for picking and placing obejcts
   */
  DeliveryTask(const std::shared_ptr<nursebot::Navigator>& navigator,
      const std::shared_ptr<nursebot::PickPlaceController>& pick_place_ctlr);

  /**
   * @brief Destroy the DeliveryTask object
   * 
   */
  virtual ~DeliveryTask();

  /**
   * @brief method to contain the logic to perform the delivery task
   * 
   * @param task_msg message object containing information about the task
   */
  bool perform_task(
      const nurse_bot::Task::ConstPtr& task_msg) override;

 private:
  /**
   * @brief pointer to navigator object responsible for moving the bot or its parts
   * 
   */
  std::shared_ptr<nursebot::Navigator> navigator;

  /**
   * @brief pointer the pick_place controller oject
   * Responsible for the picking and placing the objects
   * 
   */
  std::shared_ptr<nursebot::PickPlaceController> pick_place_ctlr;
};

}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_TASK_ACTION_HPP_
