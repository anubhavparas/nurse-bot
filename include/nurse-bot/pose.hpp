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
 * @file pose.hpp
 * @author Sakshi Kakde (sakshi@umd.edu) 
 * @author Siddharth Telang (stelang@umd.edu)
 * @author Anubhav Paras (anubhavp@umd.edu)
 * @brief Struct for storing pose information of a frame
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_NURSE_BOT_POSE_HPP_
#define INCLUDE_NURSE_BOT_POSE_HPP_

namespace nursebot {
struct Pose {
 public:
  /**
   * @brief Construct a new Pose object
   * 
   * @param x metres
   * @param y metres
   * @param z metres
   * @param roll Angle around x (rad)
   * @param pitch Angle around y (rad)
   * @param yaw Angle around z (rad)
   */
  Pose(float x, float y, float z, float roll, float pitch, float yaw) :
            x(x), y(y), z(z),
            roll(roll), pitch(pitch), yaw(yaw),
            qx(0), qy(0), qz(0), qw(1) {}

  /**
     * @brief Construct a new Pose object and store the orientation in quaternion formati
     * 
     * @param x metres
     * @param y metres
     * @param z metres
     * @param qx 
     * @param qy 
     * @param qz 
     * @param qw 
     */
  Pose(float x, float y, float z, float qx, float qy, float qz, float qw) :
            x(x), y(y), z(z),
            roll(0), pitch(0), yaw(0),
            qx(qx), qy(qy), qz(qz), qw(qw) {}

  /**
   * @brief Destroy the Pose object
   * 
   */
  ~Pose() {}

  float x, y, z, roll, pitch, yaw;

  // for storing orientation in quaternion format
  float qx, qy, qz, qw;
};
}  // namespace nursebot

#endif  // INCLUDE_NURSE_BOT_POSE_HPP_
