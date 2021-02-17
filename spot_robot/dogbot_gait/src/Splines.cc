
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

#include "ReactRobotics/GaitController/Splines.hh"

#include <assert.h>
#include <math.h>
#include <iostream>

namespace ReactRoboticsN { namespace GaitControllerN {


  SplinePoint3dC::SplinePoint3dC(float t,float x,float y,float z, SmoothingTypeT s, float smoothTime)
   : m_timeDelta(t),
     m_point(x,y,z)
  { SetupSmoothing(s, smoothTime); }

  SplinePoint3dC::SplinePoint3dC(float t,const Eigen::Vector3f &pnt, SmoothingTypeT s, float smoothTime)
   : m_timeDelta(t),
     m_point(pnt)
  { SetupSmoothing(s, smoothTime); };

  void SplinePoint3dC::SetupSmoothing(SmoothingTypeT s, float t){
    if (t <= 0.0 || s == sm_none) {
      m_smooth = sm_none;
      return;
    }
    m_smooth = s;
    if (m_smooth == sm_blend_mid || m_smooth == sm_arc) {
      m_tafter = t / 2.0;
      m_tbefore = t / 2.0;
    }
    else if (m_smooth == sm_blend_pre) {
      m_tafter = 0.0;
      m_tbefore = t;
    }
    else if (m_smooth == sm_blend_pre) {
      m_tafter = t;
      m_tbefore = 0.0;
    }
  }

  // -------------------------------------
  Spline3dC::Spline3dC()
  {}

  //! construct the spline with a set of control points; they should define a closed loop
  Spline3dC::Spline3dC(std::vector<SplinePoint3dC> &points)
  {
    Setup(points);
  }

  //! Setup control points
  void Spline3dC::Setup(const std::vector<SplinePoint3dC> &points)
  {
    m_trajectory.clear();
    float t = 0;
    for(auto &a : points) {
      m_trajectory.emplace(t,a);
      t += a.m_timeDelta;
    }
    m_totalTime = t;
    // and calculate the contact speed.  Assumes contact is between first two control points
    auto it = m_trajectory.begin();
    float tContact = it->first;
    Eigen::Vector3f start = it->second.m_point;
    ++it;
    m_contactTime = (it->first - tContact);
    m_contactDistance = (start - it->second.m_point).norm();
   // std::cout << "contactTime: " << m_contactTime << " of " << m_totalTime << ", over a distance of " << m_contactDistance << std::endl;
  }

  float Spline3dC::ContactSpeed(float omega){
    //foot moves round loop based on omega rad/sec
    float contactTime = m_contactTime / m_totalTime;
    contactTime *= (M_PI*2 / omega);
    return m_contactDistance / contactTime;
  }

  bool Spline3dC::Evaluate(float tp,Eigen::Vector3f &pnt) {
    std::cout << "Evaluate not implemented for base Spline3DC class.";
    return true;
  }

  void Spline3dC::blend(SplinePoint3dC points[3], bool beforeCorner, float t, Eigen::Vector3f &pnt)
  {
    //the points array contains the points on the current segment at 0 and 1
    //t is relative to the corner
    int bc = 0;
    if (beforeCorner){bc = 1;}
    SplinePoint3dC corner = points[bc];
    SplinePoint3dC pafter = points[bc+1];
    SplinePoint3dC pbefore = points[0];
    if (!beforeCorner){pbefore = points[2];}

    SmoothingTypeT s = corner.m_smooth;
    float delta_t = corner.m_tafter + corner.m_tbefore;
    float tnorm = (t + corner.m_tbefore) / delta_t; //proportion of the way through the blend
    float f1 = 0.0;  //fraction of first-leg blend section to apply
    float f2 = 0.0;

    if (s == sm_blend_mid){
      // this is a simple parabolic blend of two straight lines
      f1 = 1.0 - (1.0 - tnorm)*(1.0 - tnorm);
      f2 = tnorm*tnorm;

    } else if (s == sm_arc){
      // circle segments, so derivatives are 0 and 1 at either end
      // produces a nice outline, but moves rather fast at the blend edges
      f1 = std::sqrt(1.0 - (1.0 - tnorm)*(1.0 - tnorm));
      f2 = 1.0 - std::sqrt(1.0 - tnorm*tnorm);

    } else {
      std::cerr << "Error, smoothing type not implemented!"<<std::endl;
      return;
    }
    //apply the factors to the two line segments
    Eigen::Vector3f firstLeg = (corner.m_point - pbefore.m_point) * (corner.m_tbefore / (pbefore.m_timeDelta));
    Eigen::Vector3f secondLeg = (pafter.m_point - corner.m_point) * (corner.m_tafter / (corner.m_timeDelta));
#if 0
    if (debugstuff){
      std::cerr << "tnorm: " << tnorm << ", f1: " << f1 << ", f2: " << f2 << ", 1st: " << firstLeg.norm() << ", 2nd: " << secondLeg.norm() << std::endl;
      std::cerr << "blend for: " << corner.m_tbefore << " out of: " << corner.m_timeDelta << std::endl;
    }
#endif

    pnt = corner.m_point  + (f1 - 1.0)*firstLeg + f2*secondLeg;
  }

  SplinePoint3dC Spline3dC::priorPoint(float t){
    //return the previous point around the trajectory
    auto it = m_trajectory.upper_bound(t);
    if(it == m_trajectory.begin()) {
      it = m_trajectory.end();
    }
    --it;
    return it->second;
  }


  // -------------------------------------

  SplineLinear3dC::SplineLinear3dC()
  { }

  //! Construct from a list of positions
  SplineLinear3dC::SplineLinear3dC(std::vector<SplinePoint3dC> &points)
  	: Spline3dC(points)
  { }

  //! Evaluate position at time t
  bool SplineLinear3dC::Evaluate(float tp,Eigen::Vector3f &pnt)
  {
#if 1
    if(tp > m_totalTime)
      tp -= floor(tp/m_totalTime) * m_totalTime;
    if(tp < 0)
      tp += floor(tp/m_totalTime) * m_totalTime;
#endif

    auto it = m_trajectory.upper_bound(tp);

    float tOff = 0;
    // Go back 1 point.
    if(it == m_trajectory.begin()) {
      it = m_trajectory.end();
      tOff -= m_totalTime;
    }
    --it;

    float t[2];
    Eigen::Vector3f P[2];
    SplinePoint3dC SP[3]; //contains points either side of the current location plus the previous/next

    for(int i = 0;i < 2;i++) {
      t[i] = it->first + tOff;
      P[i] = it->second.m_point;
      SP[i] = it->second;
      ++it;
      if(it == m_trajectory.end()) {
        it = m_trajectory.begin();
        tOff += m_totalTime;
      }
    }
    //may need the following point for smoothing
    SP[2] = it->second;
    assert(tp >= t[0]);
    assert(tp <= t[1]);

    //do a simple linear interpolation
    pnt = P[0] * ((t[1]-tp)/(t[1]-t[0])) + P[1] * ((tp-t[0])/(t[1]-t[0]));

    //and then check for whether to apply smoothing
    if (SP[0].m_smooth != sm_none || SP[1].m_smooth != sm_none){
      //depending how close we are to the corner, may want to smooth the corners
      if (SP[0].m_smooth != sm_none && (tp - t[0]) < SP[1].m_tafter){
        //smoothing at the start of the section, need to get the prior point
        SP[2] = priorPoint(t[0]-0.0001);
        blend(SP, false,  tp - t[0], pnt);

      } else if (SP[1].m_smooth != sm_none && (t[1] - tp) < SP[1].m_tbefore ) {
        //smoothing at the end of the section, before the next corner
        blend(SP, true, tp - t[1], pnt);
      }
    } else{
      std::cerr << "skipping " << tp << ", " << SP[0].m_smooth;
    }

    return true;
  };

  // -------------------------------------

  SplineCatmullRom3dC::SplineCatmullRom3dC()
  {}

  SplineCatmullRom3dC::SplineCatmullRom3dC(std::vector<SplinePoint3dC> &points)
    : Spline3dC(points)
  {}

  bool SplineCatmullRom3dC::Evaluate(float tp,Eigen::Vector3f &pnt) {
#if 1
    if(tp > m_totalTime)
      tp -= floor(tp/m_totalTime) * m_totalTime;
    if(tp < 0)
      tp += floor(tp/m_totalTime) * m_totalTime;
#endif

    auto it = m_trajectory.upper_bound(tp);
    float tOff = 0;
    // Go back 2 points.
    for(int i = 0;i < 2;i++) {
      if(it == m_trajectory.begin()) {
        it = m_trajectory.end();
        tOff -= m_totalTime;
      }
      --it;
    }
    // Pull out points and times
    float t[4];
    Eigen::Vector3f P[4];
    for(int i = 0;i < 4;i++) {
      t[i] = it->first + tOff;
      P[i] = it->second.m_point;
      ++it;
      if(it == m_trajectory.end()) {
        it = m_trajectory.begin();
        tOff += m_totalTime;
      }
    }
    assert(tp >= t[1]);
    assert(tp <= t[2]);

    assert(t[0] < t[1]);
    assert(t[1] < t[2]);
    assert(t[2] < t[3]);

    Eigen::Vector3f A1 = P[0] * ((t[1]-tp)/(t[1]-t[0])) + P[1] * ((tp-t[0])/(t[1]-t[0]));
    Eigen::Vector3f A2 = P[1] * ((t[2]-tp)/(t[2]-t[1])) + P[2] * ((tp-t[1])/(t[2]-t[1]));
    Eigen::Vector3f A3 = P[2] * ((t[3]-tp)/(t[3]-t[2])) + P[3] * ((tp-t[2])/(t[3]-t[2]));

    Eigen::Vector3f B1 = A1 * ((t[2]-tp)/(t[2]-t[0])) + A2 * ((tp-t[0])/(t[2]-t[0]));
    Eigen::Vector3f B2 = A2 * ((t[3]-tp)/(t[3]-t[1])) + A3 * ((tp-t[1])/(t[3]-t[1]));

    pnt  = B1 * ((t[2]-tp)/(t[2]-t[1])) + B2 *((tp-t[1])/(t[2]-t[1]));

    //std::cout << "t=" << tp << " t0=" << t[0] << " t1=" << t[1] << " t2=" << t[2] << " t3=" << t[3] << "   -> " << pnt[0] << " " << pnt[2] << "\n";

    return true;
  }



}}
