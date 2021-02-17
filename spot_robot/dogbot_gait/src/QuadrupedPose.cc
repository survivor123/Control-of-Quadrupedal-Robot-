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
 */

#include "ReactRobotics/GaitController/QuadrupedPose.hh"

namespace ReactRoboticsN { namespace GaitControllerN {

  // Set the leg goal position
  void QuadrupedPoseC::SetFootPosition(int legId,float x,float y,float z)
  {
    m_feet[legId] =  Eigen::Vector3f(x,y,z);
  }

  // Set the leg goal position
  void QuadrupedPoseC::GetFootPosition(int legId,float &x,float &y,float &z) const
  {
    const Eigen::Vector3f &at = m_feet[legId];
    x = at[0];
    y = at[1];
    z = at[2];
  }

  //! Dump pose.

  void QuadrupedPoseC::Dump(std::ostream &out)
  {
    for(int i = 0;i < 4;i++) {
      out << " (" << m_feet[i] << ")";
    }
  }


  // Set the leg goal position
  void QuadrupedPoseC::SetLegPosition(int legId,const Eigen::Vector3f &at)
  {
    assert(legId >= 0 && legId < 4);
    m_feet[legId] = at;
  }

  // Set the leg goal position
  void QuadrupedPoseC::GetLegPosition(int legId,Eigen::Vector3f &at) const
  {
    assert(legId >= 0 && legId < 4);
    at = m_feet[legId];
  }

  void QuadrupedPoseC::Interpolate(
      float fract,
      const QuadrupedPoseC &nextPose,
      QuadrupedPoseC &outputPose
      )
  {
    float onemf = 1.0f-fract;
    for(int i = 0;i < 4;i++) {
      outputPose.m_feet[i] = nextPose.m_feet[i] * fract + m_feet[i] * onemf;
    }
  }


}}
