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
 * @file aruco_detector.hpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Defining the class to detect the aruco tag on the objects
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_NURSE_BOT_ARUCO_DETECTOR_HPP_
#define INCLUDE_NURSE_BOT_ARUCO_DETECTOR_HPP_

#include <fiducial_msgs/FiducialTransformArray.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

#include <array>
#include <string>
#include <memory>

namespace nursebot {
class ArucoDetector {
 public:
  /**
   * @brief Construct a new ArucoDetector object
   * 
   */
  ArucoDetector();

  /**
     * @brief Destroy the ArucoDetector object
     * 
     */
  virtual ~ArucoDetector();

  /**
   * @brief Get the object_pose object
   * 
   * @return geometry_msgs::PoseStamped 
   */
  virtual geometry_msgs::PoseStamped get_object_pose() {
    return this->object_pose;
  }

  /**
   * @brief Get the pre_grasp_pose for the arm
   * 
   * @return geometry_msgs::PoseStamped 
   */
  virtual geometry_msgs::PoseStamped get_pre_grasp_pose() {
    return this->object_offset_pose;
  }


  /**
   * @brief Get the grasp_pose for the arm
   * 
   * @return geometry_msgs::PoseStamped 
   */
  virtual geometry_msgs::PoseStamped get_grasp_pose() {
    return this->grasp_pose;
  }

  /**
   * @brief Method to check if the object is detected
   * 
   * @return true if object with aruco tag is detected
   * @return false if object with aruco taag is not detected
   */
  virtual bool is_object_detected() {
    ROS_WARN_STREAM("ArucoDetector::is_object_detected()::"
                    << "checking for detection.");
    return this->detected;
  }

  virtual void object_detected(bool detect_flag) {
    this->detected = detect_flag;
  }

 private:
  /**
   * @brief ros node handle instance pointer
   * 
   */
  ros::NodeHandle ros_nh;

  /**
   * @brief ros subscriber for the aruco detection topic
   * topic: /fiducial_transforms
   * 
   */
  ros::Subscriber aruco_sub;
  fiducial_msgs::FiducialTransform fd_tf;
  geometry_msgs::PoseStamped object_pose;

  // pre-grasping goal pose location for the arm
  geometry_msgs::PoseStamped object_offset_pose;

  // grasping goal pose location for the arm
  geometry_msgs::PoseStamped grasp_pose;
  bool detected = false;
  std::string fid_marker_topic = "fiducial_transforms";

  /**
   * @brief callback method for the aruco detection
   * 
   * @param msg message containing the detected object transforms
   */
  void detect_callback(
          const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);

  /**
   * @brief Method to set the pose of the detected object in base_link's frame
   * 1) gets the pose of the detected aruco tag and transforms into base_link's frame
   * 2) sets a pre-grasping pose (grasing + offset)
   * 3) sets the grasping pose
   * 
   */
  void set_poses();
};
}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_ARUCO_DETECTOR_HPP_
