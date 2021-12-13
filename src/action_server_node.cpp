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
 * @file action_server_node.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief ROS node to spin the TaskActionServer up
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>

#include <nurse-bot/task_action_server.hpp>

#include <nurse-bot/movebaseaction_wrapper.hpp>
#include <nurse-bot/map_navigator.hpp>
#include <nurse-bot/pick_place_controller.hpp>
#include <nurse-bot/aruco_detector.hpp>
#include <nurse-bot/robot_controller.hpp>



int main(int argc, char** argv) {
  ros::init(argc, argv, "action_server_node");
  ros::NodeHandle ros_node_h;

  std::string task_action_server_name;
  std::string move_base_server_name;
  std::string head_ctrl_name;
  std::string torso_ctrl_name;
  std::string gripper_ctrl_name;

  ros_node_h.getParam("/task_action_server_name", task_action_server_name);
  ROS_WARN_STREAM("task_action_server_name : " << task_action_server_name);

  ros_node_h.getParam("/move_base_server_name", move_base_server_name);
  ROS_WARN_STREAM("move_base_server_name : " << move_base_server_name);

  ros_node_h.getParam("/head_ctrl_name", head_ctrl_name);
  ROS_WARN_STREAM("head_ctrl_name : " << head_ctrl_name);

  ros_node_h.getParam("/torso_ctrl_name", torso_ctrl_name);
  ROS_WARN_STREAM("torso_ctrl_name : " << torso_ctrl_name);

  ros_node_h.getParam("/gripper_ctrl_name", gripper_ctrl_name);
  ROS_WARN_STREAM("gripper_ctrl_name : " << gripper_ctrl_name);


  ROS_WARN_STREAM("Initializing TaskActionServer node... ");

  std::shared_ptr<nursebot::MoveBaseActionWrapper> movebase_action =
      std::make_shared<nursebot::MoveBaseActionWrapper>(
              move_base_server_name, true);

  std::shared_ptr<nursebot::Navigator> navigator =
      std::make_shared<nursebot::MapNavigator>(movebase_action);

  nursebot::HeadControllerPtr headController =
      std::make_shared<nursebot::HeadController>(head_ctrl_name);

  nursebot::TorsoControllerPtr torsoController =
      std::make_shared<nursebot::TorsoController>(torso_ctrl_name);

  nursebot::ArmControllerPtr armController =
      std::make_shared<nursebot::ArmController>();

  nursebot::GripperControllerPtr gripperController =
      std::make_shared<nursebot::GripperController>(gripper_ctrl_name);

  std::shared_ptr<nursebot::RobotController> robotController =
      std::make_shared<nursebot::RobotController>(headController,
                                                  torsoController,
                                                  armController,
                                                  gripperController);

  std::shared_ptr<nursebot::ArucoDetector> arucoDetector =
      std::make_shared<nursebot::ArucoDetector>();

  std::shared_ptr<nursebot::PickPlaceController> pick_place_ctrl =
      std::make_shared<nursebot::PickPlaceController>(
                                                  arucoDetector,
                                                  robotController);


  std::shared_ptr<nursebot::TaskAction> guidance_task_action;
  std::shared_ptr<nursebot::TaskAction> delivery_task_action;
  guidance_task_action = std::make_shared<nursebot::GuidanceTask>(navigator);
  delivery_task_action = std::make_shared<nursebot::DeliveryTask>(
                                                            navigator,
                                                            pick_place_ctrl);

  std::unique_ptr<nursebot::TaskActionServer> action_server(
      new nursebot::TaskActionServer(task_action_server_name,
                                     guidance_task_action,
                                     delivery_task_action));

  ros::spin();
  return 0;
}
