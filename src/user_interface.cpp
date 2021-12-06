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
 * @file user_interface_node.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief ROS node for to spawn user interface and publish messages
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <nurse_bot/Task.h>
#include <nurse_bot/NBTaskAction.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <memory>
#include <sstream>

#include <nurse-bot/task_publisher.hpp>
#include <nurse-bot/task_action_client.hpp>
#include <nurse-bot/user_interface.hpp>


nursebot::UserInterface::UserInterface(std::shared_ptr<nursebot::TaskActionClient>_task_ac)
                                :task_ac(_task_ac) {}

nursebot::UserInterface::~UserInterface() {
}

void nursebot::UserInterface::get_user_input() {
  nurse_bot::NBTaskGoal task_goal;

  char continue_to_read = 'y';
  do {
    std::cout<< "Do you want Guidance or Delivery?" << std::endl;
    std::cin >> task_goal.task_type;

    std::cout << "Please provide the entity position - x,y" << std::endl;
    std::cin >> input[0] >> input[1];

    std::cout << "Please provide the target position - x,y" << std::endl;
    std::cin >> input[2] >> input[3];

    std::cout << "Sending goal to navigation server........." << std::endl;

    task_goal.task_id = "G" + std::to_string(count);

    entity_position.linear.x = 2;
    entity_position.linear.y = 2;

    entity_position.linear.x = -5;
    entity_position.linear.y = -5;

    task_goal.entity_position = this->entity_position;
    task_goal.target_position = this->target_position;

    task_ac->request_action(task_goal);

    std::cout << "Do you want to continue? y/n" << std::endl;
    std::cin >> continue_to_read;
  } while (continue_to_read == 'y' || continue_to_read == 'Y');
}
