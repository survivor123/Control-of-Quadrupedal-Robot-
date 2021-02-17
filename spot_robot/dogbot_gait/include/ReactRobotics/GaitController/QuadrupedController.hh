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
 * Base class for quadruped controllers. These controllers take the robot state
 * and output joint angles for the next position of the robot. They are generally
 * stateful.
 *
 */

#pragma once

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Geometry>
#include "ReactRobotics/GaitController/QuadrupedPose.hh"
#include "ReactRobotics/DogBotV4LegKinematics.hh"
#include <spdlog/spdlog.h>

namespace ReactRoboticsN {
  class RobotAPIC;
}

namespace ReactRoboticsN { namespace GaitControllerN {

  //! State of a quadruped robot.

  class QuadrupedStateC
  {
  public:
    //! No state known.
    QuadrupedStateC()
    {}

    //! Construct with joint information only
    QuadrupedStateC(
        const Eigen::VectorXf &jointAngle,
        const Eigen::VectorXf &jointVelocity,
        const Eigen::VectorXf &jointTorque
        )
     : m_hasJointData(true),
       m_jointAngle(jointAngle),
       m_jointVelocity(jointVelocity),
       m_jointTorque(jointTorque)
    {}

    //! Construct with joint and IMU data.
    QuadrupedStateC(
        const Eigen::VectorXf &jointAngle,
        const Eigen::VectorXf &jointVelocity,
        const Eigen::VectorXf &jointTorque,
        const Eigen::Vector3f &imuLinearAcceleration,
        const Eigen::Vector3f &imuRotationVelocity
        )
     : m_hasJointData(true),
       m_jointAngle(jointAngle),
       m_jointVelocity(jointVelocity),
       m_jointTorque(jointTorque),
       m_hasIMUData(true),
       m_imuLinearAcceleration(imuLinearAcceleration),
       m_imuRotationVelocity(imuRotationVelocity)
    {}

    bool HasJointData() const
    { return m_hasJointData; }

    const Eigen::VectorXf &JointAngle() const
    { return m_jointAngle; }

    const Eigen::VectorXf &JointVelocity() const
    { return m_jointVelocity; }

    const Eigen::VectorXf &JointTorque() const
    { return m_jointTorque; }

    bool HasIMUData() const
    { return m_hasIMUData; }

    const Eigen::Vector3f &IMULinearAcceleration() const
    { return m_imuLinearAcceleration; }

    const Eigen::Vector3f &IMURotationVelocity() const
    { return m_imuRotationVelocity; }

  protected:
    bool m_hasJointData = false;
    Eigen::VectorXf m_jointAngle;
    Eigen::VectorXf m_jointVelocity;
    Eigen::VectorXf m_jointTorque;

    bool m_hasIMUData = false;
    Eigen::Vector3f m_imuLinearAcceleration;
    Eigen::Vector3f m_imuRotationVelocity;
  };

  //! Simple Gait generator base class.
  // The controller is stateful, and may extract data from the RobotAPI.

  class QuadrupedControllerC
  {
  public:
    QuadrupedControllerC();

    //! Virtual destructor
    virtual ~QuadrupedControllerC();

    //! Set omega
    virtual void SetOmega(float omega);

    //! Do a single timestep
    // Velocity is a 6 value vector: x,y,z rotation rx,ry,rz (=pitch,roll,yaw).
    // Some rotations or movements may not be supported.
    virtual bool Step(
        float timeStep,
        const QuadrupedStateC &state,
        const Eigen::VectorXf &velocity,
        Eigen::VectorXf &nextJointAngles
        );

    //! Do a single timestep
    // Velocity is a 6 value vector: x,y,z rotation rx,ry,rz.
    // This does a step with a forward velocity of 1 m/s.   Vector: 0,1,0,0,0,0
    bool Step(
        float timeStep,
        const QuadrupedStateC &state,
        Eigen::VectorXf &nextJointAngles
        );

    //! Set the gait style
    virtual bool SetStyle(const std::string &styleName);

    //! Access kinematics being used.
    DogBotV4KinematicsC &Kinematics()
    { return m_kinematics; }

    //! Access kinematics being used.
    const DogBotV4KinematicsC &Kinematics() const
    { return m_kinematics; }

    //! Convert pose to joint angles.
    bool Pose2JointAngles(
        const QuadrupedPoseC &positions,
        Eigen::VectorXf &nextJointAngles
        );

    //! Convert joint angles to pose
    bool JointAngles2Pose(
        const Eigen::VectorXf &nextJointAngles,
        QuadrupedPoseC &positions
        );

    //! Access ordered list of expected joint names used in the controller.
    std::vector<std::string> JointNames() const
    { return m_kinematics.JointNames(); }

    //! Set logger to use use.
    void SetLog(std::shared_ptr<spdlog::logger> log)
    { m_log = log; }

    //! Used to plot gait information, the exact output depends on the controller.
    virtual void PlotGait();

  protected:
    DogBotV4KinematicsC m_kinematics;
    Eigen::VectorXf m_velocity; // Velocity is a 6 value vector: x,y,z rotation rx,ry,rz (=pitch,roll,yaw).
    float m_omega = 2;  //!< Radians / second cycle speed
    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");

  };

}}

