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
 * The SplineGaitControllerC class is derived from the QuadrupedControllerC
 * base class, and uses the FootTrajectory class to encapsulate the key parameters
 * for each foot's path. Most coordinates are in the local leg coordinate system
 *
 * The controller contains a vector of four SplineGait objects, m_footTrajectories
 * (SplineLinear3dC or SplineCatmullRom3dC) to control each leg
 *
 */

#pragma once

#include <array>
#include "ReactRobotics/GaitController/QuadrupedController.hh"
#include "Splines.hh"

namespace ReactRoboticsN { namespace GaitControllerN {

  enum SplineGaitTypeT
  {
    SGT_Walk,
    SGT_Trot
  };

  //! Foot trajectory parameters.
  // From 'Locomotion Control for Electrically Powered Quadruped Robot Dynarobin'
  // by Edin Koco

  class FootTrajectoryC
  {
  public:
    FootTrajectoryC();

    //! Setup trajectory
    FootTrajectoryC(float Zc,float Xc,float Lpr,float tpr,float Had,float tad,float tpu);

    std::vector<SplinePoint3dC> GenerateTrajectory(float rz,float xoff,float zoff) const;

    float m_Zc = 0.01;  // z centre
    float m_Xc = -0.06;  // x centre
    float m_Lpr = 0.15; // Length propel
    float m_tpu = 0.1;  // Time push segment

    float m_Had = 0.2; // Height adjust
    float m_tad = 0.2; // Time adjust segment
    float m_tpr = 0.5; // Time propel

    float m_Rpu = 0.0;//0.03; // Distance push
    float m_apu = 0.8; // Angle push

    //default smoothing to apply
    float m_smoothingPeriod = 0.03;
    SmoothingTypeT m_smooth = sm_blend_mid;

  };

  //! Gait generator class using a 4-phase spline construction for each foot's motion.

  class SplineGaitControllerC
    : public QuadrupedControllerC
  {
  public:
    SplineGaitControllerC();

    //! Do a single timestep
    // Velocity is a 6 value vector: x,y,z rotation rx,ry,rz.
    // Some rotations or movements may not be supported.
    bool Step(
        float timeStep,
        const QuadrupedStateC &state,
        const Eigen::VectorXf &velocity,
        Eigen::VectorXf &nextJointAngles
        ) override;

    //! Set the gait style
    bool SetStyle(const std::string &styleName) override;

    //! Plot graph of gait
    void PlotGait() override;

  protected:
    bool m_haveInitialPose = false;

    void GenerateFootTrajectory(enum SplineGaitTypeT gaitType);

    float m_maxSpeed = 6;

    float m_zOffset = 0.4;

    float m_zCentre = 0.0;
    float m_defaultZcentre = 0.0;
    float m_zCentreRange = 0.15;

    float m_Xcentre = -0.00;
    float m_defaultXcentre = -0.06;
    float m_xCentreRange = 0.15;

    float m_timePropel = 0.5;
    float m_defaultTimePropel = 0.5;
    float m_minTimePropel = 0.05;
    float m_maxTimePropel = 1.0;

    float m_hightAdjust = 0.3;
    float m_defaultHightAdjust = 0.15;
    float m_maxHightAdjust = 0.23;

    float m_lengthPropel = 0.15;
    float m_defaultLengthPropel = 0.1;
    float m_maxlengthPropel = 0.16;

    float m_footRotate = 0;
    float m_footRotateMax = M_PI/2;

    float m_footSeperation = 0.0;
    float m_footSeperationDefault = 0.05;
    float m_footSeperationRange = 0.2;

    float m_tiltX = 0;
    float m_tiltXRange = 0.15;

    float m_tiltY = 0;
    float m_tiltYRange = 0.15;

    float m_phase = 0;  //!< Current phase in radians
    float m_defaultOmega = 4;  //!< Radians / second cycle speed

    float m_phases[4] = { 0,0,0,0 };

    //body width and length to the leg origins, not total chassis size
    //later- set from urdf?
    float m_bodyWidth = 0.5; //0.374;  //to the leg centres, when hips are flat
    float m_bodyLength = 0.559;  //to hip centrelines, constant

    float m_blendTime = 0.5;
    float m_blendAlpha =1.0;


    std::array<Eigen::Vector3f,4> m_initialFootLocations;

    std::array<Eigen::Vector3f,4> m_legOrigins; //location of the shoulders relative to the body

    std::vector<SplineLinear3dC> m_footTrajectories;
    //std::vector<SplineCatmullRom3dC> m_footTrajectories;

  };

}}

