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
 * ---------------------------------------------------------------------
 *
 * Robot kinematics
 */

#pragma once

#include <jsoncpp/json/json.h>
#include <cassert>
#include <eigen3/Eigen/Geometry>
#include <array>

namespace ReactRoboticsN
{


  //! This computes the position of the end effector given the end effector

  // l1 = Upper leg length.
  // l2 = Lower leg length

  // Coordinates:
  //  x = sideways    -v left / +v right
  //  y = forward/back +v / -v
  //  z = Height above ground
  //
  // Angles:
  //  0 - Roll
  //  1 - Pitch
  //  2 - Knee
  //

  class DogBotV4LegKinematicsC
  {
  public:
    //! Default constructor
    DogBotV4LegKinematicsC();

    //! Construct from a json object
    DogBotV4LegKinematicsC(const Json::Value &value);

    //! Access name of leg.
    const std::string &Name() const
    { return m_name; }

    //! Set name of leg.
    void SetName(const std::string &name)
    { m_name = name; }

    //! Configure from JSON
    bool ConfigureFromJSON(const Json::Value &value);

    //! Get the servo configuration as JSON
    Json::Value ConfigAsJSON() const;

    //! Clip foot position to nearest valid location
    // Returns true if within bound and false if clipped.
    bool ClipPosition(Eigen::Vector3f &position) const;

    //! Inverse kinematics for the leg
    //! Compute joint angles needed to get to a 3d position in robot coordinate system (Provided origin is set)
    //! Return true if position is reachable
    bool Inverse(const Eigen::Vector3f &position,Eigen::Vector3f &angles) const;

    //! Forward kinematics for the leg
    //! Compute the position of the foot relative to the robot centre from the joint angles.
    bool Forward(const Eigen::Vector3f &angles,Eigen::Vector3f &position) const;


    // ! Access joint directions
    float JointDirection(int jnt) const
    {
      assert(jnt >= 0 && jnt < 3);
      return m_jointDirections[jnt];
    }

    //! Access joint directions as vector
    const Eigen::Vector3f &JointDirections() const
    { return m_jointDirections; }

    //! Set joint directions
    void SetJointDirections(const Eigen::Vector3f &jd)
    { m_jointDirections = jd; }

    //! Offset from roll axis of rotation
    float LengthHipX() const
    { return m_l0; }

    void InvertL0()
    { m_l0 *= -1; }

    //! Length of upper leg
    float LengthUpperLeg() const
    { return m_l1; }

    //! Length of lower leg
    float LengthLowerLeg() const
    { return m_l2; }

    //! Offset from roll axis of rotation
    float LengthZDrop() const
    { return 0; }

    //! Distance bellow the pivot of the centre of the foot
    float LengthFootDrop() const
    { return m_footDrop; }

    //! Radius of foot sphere
    float FootSphereRadius() const
    { return m_footSphereRadius; }

    //! Leg origin.
    float LegOrigin(int coordinate) const
    { return m_legOrigin[coordinate]; }

    //! Leg origin.
    const Eigen::Vector3f &Origin() const
    { return m_legOrigin; }

    //! Compute the maximum extension of the leg vertically down
    float MaxExtension() const
    { return m_maxExtension; }

    //! Compute the minimum extension of the leg vertically down
    float MinExtension() const
    { return m_minExtension; }

    //! Compute the maximum stride length at a given leg extension.
    float StrideLength(float extension) const;

    //! Compute an estimate of the force on a foot and where it is given some angles and torques
    //! Not working yet
    bool ComputeFootForce(
        const Eigen::Vector3f &angles,
        const Eigen::Vector3f &jointVelocities,
        const Eigen::Vector3f &torques,
        Eigen::Vector3f &position,
        Eigen::Vector3f &velocity,
        Eigen::Vector3f &force
        ) const;

    //! Set leg origin
    void SetOrigin(const Eigen::Vector3f &origin)
    { m_legOrigin = origin; }

    //! Leg origin
    const Eigen::Vector3f &LegOrigin() const
    { return m_legOrigin; }
  protected:


    void Init();

    std::string m_name; // Leg name

    float m_minExtension = -1;
    float m_maxExtension = -1;

    Eigen::Vector3f m_legOrigin = { 0, 0, 0};
    Eigen::Vector3f m_jointDirections = { 1.0, 1.0, 1.0 };

    float m_l0 = 0.09875; // Hip offset in x axis
    float m_l1 = 0.315; // Upper leg length
    float m_l2 = 0.30;  // Lower leg length

    float m_footDrop = 0.025; // Distance of centre of foot bellow
    float m_footSphereRadius = 0.053; // Radius of foot sphere

  };

  //! Kinematics for whole robot.

  class DogBotV4KinematicsC
  {
  public:
    DogBotV4KinematicsC();

    //! Access legs
    DogBotV4LegKinematicsC &LegKinimatics(int i)
    { return m_legKinimatics[i]; }

    //! Convert array of 4 feet positions in the robot body coordinate frame to joint angles, in the order given by 'JointNames()'
    bool FeetPositions2JointAngles(const std::array<Eigen::Vector3f,4> &feetPositions,Eigen::VectorXf &angles);

    //! Convert joint angles to feet position
    // Positions must be an array of at least 4 Vector3f's
    bool JointAngles2FeetPositions(const Eigen::VectorXf &angles,std::array<Eigen::Vector3f,4> &feetPositions);

    //! Clip foot positions to nearest valid location
    // Returns true if within bound and false if clipped.
    bool ClipPositions(std::array<Eigen::Vector3f,4> &feetPositions) const;

    //! Get origin for leg.
    const Eigen::Vector3f &LegOrigin(int legId) const
    { return m_legKinimatics[legId].Origin(); }

    //! Access a list of joint names
    static std::vector<std::string> JointNames();

  protected:
    //! Names of joints
    static const char *m_jointNames[];

    // These are used to compute the leg positions, though they don't really belong here.
    float m_bodyWidth = 0.304;
    float m_bodyLength = 0.556;

    DogBotV4LegKinematicsC m_legKinimatics[4];
  };


}

