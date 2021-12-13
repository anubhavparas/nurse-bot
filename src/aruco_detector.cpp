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

#include <nurse-bot/aruco_detector.hpp>

nursebot::ArucoDetector::ArucoDetector() {
  this->aruco_sub = ros_nh.subscribe(
      this->fid_marker_topic,
      10,
      &ArucoDetector::detect_callback,
      this);
  ROS_WARN_STREAM("ArucoDetector():: Subscriber initialized..");
}

nursebot::ArucoDetector::~ArucoDetector() {
}

void nursebot::ArucoDetector::detect_callback(
    const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
  if (!this->detected && msg->transforms.size() > 0) {
    this->fd_tf = msg->transforms[0];
    this->detected = true;
    this->set_poses();
  }
}

void nursebot::ArucoDetector::set_poses() {
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);

  geometry_msgs::TransformStamped camera_to_base_link_tf =
    tf_buffer.lookupTransform(
      "base_footprint",
      "xtion_rgb_optical_frame",
      ros::Time(0),
      ros::Duration(10.0));

  // set the pose values of the object as per the fiducial transform msg
  this->object_pose.pose.position.x = fd_tf.transform.translation.x;
  this->object_pose.pose.position.y = fd_tf.transform.translation.y;
  this->object_pose.pose.position.z = fd_tf.transform.translation.z;

  // transform from marker/camera frame to base_link frame
  tf2::doTransform(
    this->object_pose.pose,
    this->object_pose.pose,
    camera_to_base_link_tf);


  // set the pre-grasp pose values for the arm
  this->object_offset_pose.pose.position.x =
                  this->object_pose.pose.position.x - 0.1;
  this->object_offset_pose.pose.position.y =
                  this->object_pose.pose.position.y;
  this->object_offset_pose.pose.position.z =
                  this->object_pose.pose.position.z + 0.3;

  this->object_offset_pose.pose.orientation.x = 0.707;
  this->object_offset_pose.pose.orientation.y = 0.0;
  this->object_offset_pose.pose.orientation.z = 0.0;
  this->object_offset_pose.pose.orientation.w = 0.707;


  // set the grasp pose values for the arm
  this->grasp_pose.pose.position.x =
                  this->object_pose.pose.position.x - 0.1;
  this->grasp_pose.pose.position.y =
                  this->object_pose.pose.position.y;
  this->grasp_pose.pose.position.z =
                  this->object_pose.pose.position.z;

  this->grasp_pose.pose.orientation.x = 0.707;
  this->grasp_pose.pose.orientation.y = 0.0;
  this->grasp_pose.pose.orientation.z = 0.0;
  this->grasp_pose.pose.orientation.w = 0.707;

  ROS_WARN_STREAM("ArucoDetector::set_poses():: object pose: "
                  << "x: " << object_pose.pose.position.x
                  << ", y: " << object_pose.pose.position.y
                  << ", z: " << object_pose.pose.position.z);

  ROS_WARN_STREAM("ArucoDetector::set_poses():: object pregrasp pose: "
                  << "x: " << object_offset_pose.pose.position.x
                  << ", y: " << object_offset_pose.pose.position.y
                  << ", z: " << object_offset_pose.pose.position.z);

  ROS_WARN_STREAM("ArucoDetector::set_poses():: object grasp_pose pose: "
                  << "x: " << grasp_pose.pose.position.x
                  << ", y: " << grasp_pose.pose.position.y
                  << ", z: " << grasp_pose.pose.position.z);
}
