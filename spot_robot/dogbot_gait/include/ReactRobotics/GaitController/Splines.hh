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
 * Code to produce a smoothed trajectory around a series of points
 * for each foot's motion,
 *
 * Contains a SplineLinear3dC class, which applies linear motion between
 * sections, plus optional smoothing at the section boundaries
 *
 * SplineCatmullRom3dC uses Catmull Rom spline methodology, see
 * https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Catmull%E2%80%93Rom_spline;
 *
 * The classes contain a map (ControlPoints) of SplinePoint3dC objects,
 * to define the control points around each leg's loop.  The map is
 * keyed off a float representing the time point around the loop
 *
 */

#pragma once

#include <map>
#include <vector>
#include <eigen3/Eigen/Geometry>


namespace ReactRoboticsN { namespace GaitControllerN {

  enum SmoothingTypeT { sm_none, sm_arc, sm_blend_mid, sm_blend_pre, sm_blend_post };

  //! Control point for spline
  //! points are set by FootTrajectoryC::GenerateTrajectory for each leg

  class SplinePoint3dC
  {
  public:
    SplinePoint3dC()
    {}

    SplinePoint3dC(float t,float x,float y,float z, SmoothingTypeT s=sm_none, float smoothTime=0);

    SplinePoint3dC(float t,const Eigen::Vector3f &pnt, SmoothingTypeT s=sm_none, float smoothTime=0);

    float m_timeDelta = 1.0; // Time to next point
    Eigen::Vector3f m_point;

    void SetupSmoothing(SmoothingTypeT s, float t);

    SmoothingTypeT m_smooth = sm_none; //smooth the transition around the point
    float m_tafter = 0.0; //time after the point over which to apply smoothing
    float m_tbefore = 0.0;
    Eigen::Vector3f m_pafter; //end of the smoothing phase
    Eigen::Vector3f m_pbefore;
  };

  //! Spline evaluation
  // This spline wraps around forming a closed loop

  class Spline3dC
  {
  public:
    Spline3dC();

    //! Construct from a list of positions
    Spline3dC(std::vector<SplinePoint3dC> &points);

    //! Setup control points
    void Setup(const std::vector<SplinePoint3dC> &points);

    //! Evaluate position at time t
    bool Evaluate(float t,Eigen::Vector3f &pnt);

    //! Access total time
    float TotalTime() const
    { return m_totalTime; }

    //! Access contact speed
    float ContactSpeed(float omega);

    std::map<float,SplinePoint3dC> &ControlPoints()
    { return m_trajectory; }

    void blend(SplinePoint3dC points[3], bool beforeCorner, float t, Eigen::Vector3f &pnt);

  protected:
    SplinePoint3dC priorPoint(float t);

    float m_totalTime = 0;
    std::map<float,SplinePoint3dC> m_trajectory;
    float m_contactTime = 0;
    float m_contactDistance = 0;
  };

  //! define foot trajectories based around simple movements between control points
  class SplineLinear3dC
	: public Spline3dC
  {
  public:
    SplineLinear3dC();

    //! Construct from a list of positions
    SplineLinear3dC(std::vector<SplinePoint3dC> &points);

    bool Evaluate(float t,Eigen::Vector3f &pnt);
  };

  //! use Catmull-Rom spline to smooth the trajectory.
  //! Caution, can produce undesirable features in the resulting path

  class SplineCatmullRom3dC
    : public Spline3dC
  {
  public:
    SplineCatmullRom3dC();

    //! Construct from a list of positions
    SplineCatmullRom3dC(std::vector<SplinePoint3dC> &points);

    bool Evaluate(float t,Eigen::Vector3f &pnt);
  };

}}

