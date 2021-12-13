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
 * @file test_aruco_detector.cpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Testing of ArucoDetector class
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <memory>
#include <nurse-bot/aruco_detector.hpp>

using ::testing::_;


TEST(ArucoDetectorTest, test_detection_detected) {
  ros::NodeHandle n;

  fiducial_msgs::FiducialTransformArray fid_mark_msg;
  fiducial_msgs::FiducialTransform fid_tf;
  fid_tf.transform.translation.x = 1.0;
  fid_tf.transform.translation.y = 1.0;
  fid_tf.transform.translation.z = 1.0;

  fid_mark_msg.transforms.push_back(fid_tf);

  ros::Publisher fid_mark_pub = n.advertise<
            fiducial_msgs::FiducialTransformArray>("fiducial_transforms", 10);

  nursebot::ArucoDetector arucoDetector;

  ros::WallDuration(10.0).sleep();
  ros::spinOnce();

  int count = 0;
  while (!arucoDetector.is_object_detected() && count < 100) {
    fid_mark_pub.publish(fid_mark_msg);
    count++;
    ros::WallDuration(1.0).sleep();
    ros::spinOnce();
  }

  EXPECT_TRUE(arucoDetector.is_object_detected());

  EXPECT_NEAR(arucoDetector.get_pre_grasp_pose().pose.position.x + 0.1,
              arucoDetector.get_object_pose().pose.position.x,
              0.001);
  EXPECT_NEAR(arucoDetector.get_pre_grasp_pose().pose.position.z - 0.3,
              arucoDetector.get_object_pose().pose.position.z,
              0.001);
}

TEST(ArucoDetectorTest, test_detection_not_detected) {
  ros::NodeHandle n;

  fiducial_msgs::FiducialTransformArray fid_mark_msg;
  ros::Publisher fid_mark_pub = n.advertise<
            fiducial_msgs::FiducialTransformArray>("fiducial_transforms", 10);

  nursebot::ArucoDetector arucoDetector;

  ros::WallDuration(10.0).sleep();
  ros::spinOnce();

  int count = 0;
  while (!arucoDetector.is_object_detected() && count < 10) {
    fid_mark_pub.publish(fid_mark_msg);
    count++;
    ros::WallDuration(1.0).sleep();
    ros::spinOnce();
  }

  EXPECT_FALSE(arucoDetector.is_object_detected());
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_test_node");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

