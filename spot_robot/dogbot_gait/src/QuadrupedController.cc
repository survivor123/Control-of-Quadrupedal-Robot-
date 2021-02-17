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

#include <cassert>
#include "ReactRobotics/GaitController/QuadrupedController.hh"

namespace ReactRoboticsN { namespace GaitControllerN {

  QuadrupedControllerC::QuadrupedControllerC()
  {
    Eigen::VectorXf vel(6);
    vel.fill(0);
    vel[1] = 1.0;
    m_velocity = vel;
  }

  //! Virtual destructor
  QuadrupedControllerC::~QuadrupedControllerC()
  {}

  //! Set omega
  void QuadrupedControllerC::SetOmega(float omega)
  {
    m_omega = omega;
  }

  //! Set the gait style
  bool QuadrupedControllerC::SetStyle(const std::string &styleName)
  {
    return false;
  }

  bool QuadrupedControllerC::Pose2JointAngles(
      const QuadrupedPoseC &pose,
      Eigen::VectorXf &nextJointAngles
      )
  {
    return m_kinematics.FeetPositions2JointAngles(pose.m_feet,nextJointAngles);
  }

  //! Convert joint angles to pose
  bool QuadrupedControllerC::JointAngles2Pose(
      const Eigen::VectorXf &nextJointAngles,
      QuadrupedPoseC &pose
      )
  {
    return m_kinematics.JointAngles2FeetPositions(nextJointAngles,pose.m_feet);
  }


  //! Do a single timestep
  bool QuadrupedControllerC::Step(
      float timeStep,
      const QuadrupedStateC &state,
      const Eigen::VectorXf &velocity,
      Eigen::VectorXf &nextJointAngles
      )
  {
    assert(0);
    return false;
  }

  //! Do a single timestep
  // Velocity is a 6 value vector: x,y,z rotation rx,ry,rz.
  // This does a step with a forward velocity of 1 m/s.   Vector: 0,1,0,0,0,0

  bool QuadrupedControllerC::Step(
      float timeStep,
      const QuadrupedStateC &state,
      Eigen::VectorXf &nextJointAngles
      )
  {
    return Step(timeStep,state,m_velocity,nextJointAngles);
  }

  void QuadrupedControllerC::PlotGait()
  {
    m_log->error("PlotGait not implemented for base quadruped controller");
    assert(0);
  }


}}
