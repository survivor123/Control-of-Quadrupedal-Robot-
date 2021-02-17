/*
 * MIT License
 *
 * Copyright (c) 2019 React AI Ltd.
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
 * -----------------------------------------------------------------------------
 *
 * Class to hold the pose of the robot in terms of feet position in euclidean space,
 * in the robot coordinate system.
 *
 */

#pragma once

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Geometry>
#include <array>

namespace ReactRoboticsN { namespace GaitControllerN {

  // Cartesian positions for feet in body a coordinate system
  //
  //  0-Front left  1-Front right
  //  2-Rear left   3-Read right

  class QuadrupedPoseC
  {
  public:
    // Set the leg goal position
    void SetFootPosition(int legId,float x,float y,float z);

    // Get the leg goal position
    void GetFootPosition(int legId,float &x,float &y,float &z) const;

    // Set the leg goal position
    void SetLegPosition(int legId,const Eigen::Vector3f &at);

    // Get the leg goal position
    void GetLegPosition(int legId,Eigen::Vector3f &at) const;

    // Get the leg goal position
    const Eigen::Vector3f &FootPosition(int legId) const {
      assert(legId >= 0 && legId < 4);
      return m_feet[legId];
    }

    // Interpolate between poses
    // when fact == 0, this pose is assigned to outputPose.
    // when fact == 1, then 'nextPose' is assigned to outputPose.
    void Interpolate(float fract,const QuadrupedPoseC &nextPose,QuadrupedPoseC &outputPose);

    //! Dump pose
    void Dump(std::ostream &out);

    std::array<Eigen::Vector3f,4> m_feet;
  };

}}

