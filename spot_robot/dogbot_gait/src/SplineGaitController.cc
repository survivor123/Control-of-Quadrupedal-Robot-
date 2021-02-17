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

#include "ReactRobotics/GaitController/SplineGaitController.hh"
#include "ReactRobotics/DogBotV4LegKinematics.hh"

#include <fstream>

namespace ReactRoboticsN { namespace GaitControllerN {

  FootTrajectoryC::FootTrajectoryC()
  {

  }

  //! Setup trajectory
  FootTrajectoryC::FootTrajectoryC(float Zc,float Xc,float Lpr,float tpr,float Had,float tad,float tpu)
    : m_Zc(Zc),
      m_Xc(Xc),
      m_Lpr(Lpr),
      m_tpu(tpu),
      m_Had(Had),
      m_tad(tad),
      m_tpr(tpr)
  {}

  //! create the control points around one foot's trajectory
  std::vector<SplinePoint3dC> FootTrajectoryC::GenerateTrajectory(float rz,float xoff,float zoff) const
  {
    std::vector<SplinePoint3dC> pnts;
    pnts.reserve(8);

    float Lpu = cos(m_apu) * m_Rpu;
    float Zpu = sin(m_apu) * m_Rpu;

    float LpuFin = 1.333 * Lpu;
    float Sl = LpuFin + m_Lpr; // Total stride length
    float Oad = -Sl / 7.0; // Was 9. what fraction of distance to move the half the adjustment vertical distance
    float nXc = m_Xc - Sl/2.0;
    SmoothingTypeT ms = m_smooth;
    float sp = m_smoothingPeriod;

    pnts.push_back(SplinePoint3dC(m_tpr         ,nXc + m_Lpr          ,xoff ,-(zoff + m_Zc), ms, sp));        // 1 = start of contact phase
    if(m_Rpu > 0) {

      //float timePush = m_Lpr
      pnts.push_back(SplinePoint3dC(2.0*m_tpu/3.0 ,nXc                ,xoff ,-(zoff + m_Zc), ms, sp));      // 2 = end of ground-contact
      pnts.push_back(SplinePoint3dC(m_tpu / 3.0   ,nXc - Lpu          ,xoff ,-(zoff + m_Zc-Zpu), ms, sp));  // 3
    }

    pnts.push_back(SplinePoint3dC(m_tad/4.0     ,nXc - LpuFin         ,xoff ,-(zoff + m_Zc), ms, sp));        // 4
    pnts.push_back(SplinePoint3dC(m_tad/4.0     ,nXc - LpuFin + Oad   ,xoff ,-(zoff + m_Zc+m_Had/2.0), ms, sp));// 5
    pnts.push_back(SplinePoint3dC(m_tad/4.0     ,nXc + m_Lpr - Sl/2.0 ,xoff ,-(zoff + m_Zc+m_Had), ms, sp));    // 6
    pnts.push_back(SplinePoint3dC(m_tad/4.0     ,nXc + m_Lpr - Oad    ,xoff ,-(zoff + m_Zc+m_Had/2.0), ms, sp));// 7

    for(auto &a : pnts) {
      Eigen::Vector3f op = a.m_point;
      a.m_point[0] = op[0] * cos(rz) - op[1] * sin(rz);
      a.m_point[1] = op[0] * sin(rz) + op[1] * cos(rz);
      a.m_point[2] = op[2];
    }

#if 0
    // Resample
    {
      SplineLinear3dC linear(pnts);

      std::vector<SplinePoint3dC> newPnts;
      int num = 100;
      float timeInc = linear.TotalTime()/num;
      float t = 0;
      for(int i = 0;i < num;i++,t+=timeInc) {
        Eigen::Vector3f pnt;
        linear.Evaluate(t,pnt);
        newPnts.push_back(SplinePoint3dC(t,pnt));
      }
      pnts = newPnts;
    }
#endif

    return pnts;
  }

  // ----------------------------------------------------------

  SplineGaitControllerC::SplineGaitControllerC()
   : m_footTrajectories(4)
  {
    m_blendAlpha = 0.0;
    GenerateFootTrajectory(SGT_Trot);
  }

  //! create the basic foot trajectories, called once at setup
  void SplineGaitControllerC::GenerateFootTrajectory(enum SplineGaitTypeT gaitType)
  {
    float hightAdjust = 0.2;
    float timePush = 0.02;

    switch(gaitType)
    {
      case SGT_Walk: {
        // Walk
        m_lengthPropel = 0.25;
        m_timePropel = 0.75;
        hightAdjust = 0.15;
        timePush = 0.02;

        const int LegFL=0; // 1
        const int LegBL=2; // 2
        const int LegFR=1; // 3
        const int LegBR=3; // 4

        m_phases[LegFL] = 0;
        m_phases[LegBR] = 3* M_PI/2.0;
        m_phases[LegFR] = M_PI;
        m_phases[LegBL] = M_PI/2.0;

        //m_omega = 1;
      } break;
      case SGT_Trot: {
        // Trot
        m_timePropel = 0.5;
        m_lengthPropel = 0.15;

        m_phases[0] = 0;
        m_phases[1] = M_PI;
        m_phases[2] = M_PI;
        m_phases[3] = 0;

        //m_omega = 6;

      } break;
      default:
        assert(0);
        m_log->error("Unrecognised gait type, ignoring. ");
        return ;
    }

    FootTrajectoryC trajectory(
        m_zCentre, // Zc,  Z center
        m_Xcentre, // Xc,  X center
        m_lengthPropel,  // Lpr,
        m_timePropel,   // tpr,
        m_hightAdjust,   // tad, Time adjust
        hightAdjust,   // Had, hight adjust
        timePush    // Tpu, Time push
        );

    m_footTrajectories[0].Setup(trajectory.GenerateTrajectory(-m_footRotate,-m_footSeperation,m_tiltX + m_tiltY));
    m_footTrajectories[1].Setup(trajectory.GenerateTrajectory(-m_footRotate,m_footSeperation,m_tiltX - m_tiltY));
    m_footTrajectories[2].Setup(trajectory.GenerateTrajectory(m_footRotate,-m_footSeperation,-m_tiltX + m_tiltY));
    m_footTrajectories[3].Setup(trajectory.GenerateTrajectory(m_footRotate,m_footSeperation,-m_tiltX - m_tiltY));

    m_legOrigins[0]=Eigen::Vector3f(-m_bodyWidth/2.0,m_bodyLength/2.0,0);
    m_legOrigins[1]=Eigen::Vector3f(m_bodyWidth/2.0,m_bodyLength/2.0,0);
    m_legOrigins[2]=Eigen::Vector3f(-m_bodyWidth/2.0,-m_bodyLength/2.0,0);
    m_legOrigins[3]=Eigen::Vector3f(m_bodyWidth/2.0,-m_bodyLength/2.0,0);

  }


  //! Set the gait style
  bool SplineGaitControllerC::SetStyle(const std::string &styleName)
  {
    //std::cerr << "Updating gait style: '" << styleName << "' " << std::endl;
    if(styleName == "walk") {
      GenerateFootTrajectory(SGT_Walk);
      return true;
    }
    if(styleName == "trot") {
      GenerateFootTrajectory(SGT_Trot);
      return true;
    }
    m_log->error("Unknown gait style: '{}' ",styleName);
    return false;
  }

  void SplineGaitControllerC::PlotGait() {

    int numPnts = 600;

    {
      std::ofstream plot("points.csv");
      plot << "T,X,Y,Z,timeDelta" << std::endl;

      auto &pmap = m_footTrajectories[0].ControlPoints();
      for(auto it = pmap.begin();it != pmap.end();++it) {
        Eigen::Vector3f pnt = it->second.m_point;
        plot << it->first << "," << pnt[0] << "," << pnt[1] << "," << pnt[2] << "," << it->second.m_timeDelta << std::endl;
      }
    }

    {
      std::ofstream plot("gait.csv");
      plot << "T,X,Y,Z" << std::endl;

      float totalTime = m_footTrajectories[0].TotalTime();
      float timeStep = totalTime/numPnts;
      std::cout <<"about to plot gait with interval " << timeStep << "   ";
      for(float t = 0;t < totalTime;t+= timeStep) {
        Eigen::Vector3f pnt;
        m_footTrajectories[0].Evaluate(t,pnt);
        plot << t << "," << pnt[0] << ","  << pnt[1] << "," << pnt[2] << std::endl;
      }
    }

  }

  //! Do a single timestep
  bool SplineGaitControllerC::Step(
      float timeStep,
      const QuadrupedStateC &state,
      const Eigen::VectorXf &velocity,
      Eigen::VectorXf &nextJointAngles
      )
  {
    QuadrupedPoseC pose;
#if 1
    // The initial position of the robot is assumed to be a resting position.
    // record this location so we can blend into it as the gate starts.
    if(!m_haveInitialPose && state.HasJointData()) {
      m_kinematics.JointAngles2FeetPositions(state.JointAngle(),m_initialFootLocations);
      m_haveInitialPose = true;
    }
#endif

    assert(timeStep > 0);
    m_phase += m_omega * timeStep;
    if(m_phase > M_PI*2)
      m_phase -= M_PI*2;

    // Sort out velocity request.

    // Blend velocity changes slowly.
    m_velocity += (velocity - m_velocity) * timeStep * 10;
    //std::cout << "Vel: " << m_velocity[0] << " "   << m_velocity[1] << " " << m_velocity[2] << " R:"<< m_velocity[5] << " " << std::endl;

    // Height change

    float newZoffset = m_zOffset + m_velocity[2] * timeStep;
    if(newZoffset > 0.35 && newZoffset < 0.5)
    {
      m_zOffset = newZoffset;
    }

    for(int i = 0;i < 4;i++) {
      //get the proportion around the loop for each leg in turn
      float phase = m_phase + m_phases[i];
      if(phase > M_PI*2)
        phase -= M_PI*2;
      if(phase < 0)
        phase += M_PI*2;

      //phase runs from 0 to 2pi; convert to t in seconds around the loop
      float t = phase * m_footTrajectories[i].TotalTime() / (M_PI*2.0);

      Eigen::Vector3f pnt;
      //calculate the location of the foot in its baseline trajectory
      //relative to the position specified in m_legOrigins
      //returns vectors, hence [x,y,z], but effectively it returns [horizontal,0,vertical]
      m_footTrajectories[i].Evaluate(t,pnt);
      float xVel = m_velocity[0]; //xVel and yVel in m/s
      float yVel = m_velocity[1];

      float yaw = m_velocity[5];
      if (abs(yaw) > 0.0001){
        // Add lateral motion to achieve rotation around the z-axis (zx = yaw = pnt[5])
        // at a tangent to a circle around the centre-point
        float xOffset = m_legOrigins[i][0];
        float yOffset = m_legOrigins[i][1];  // TODO - use actual x-distance based on hip roll and leg position

        xVel -= yOffset * yaw; //total add'nal velocity is sqrt(x-off^2+y-off^2), i.e. yaw*distance from centre
        yVel += xOffset * yaw;
        //std::cout << "xOffset: " << xOffset << ", yOff: " << yOffset << ", yaw: " << yaw  << std::endl;
        //possible todo - overall velocity tangential to instantaneous centre of motion of robot's path?
      }

      /* Work out the desired positions based on requested velocity
       just scale out the baseline trajectory; the "x" component from
       the base trajectory is used for both forward (y) and lateral (x) motion
       as pnt[1] will be zero with current trajectory generation
      */
      float cs = m_footTrajectories[i].ContactSpeed(m_omega);  //scale by the reference speed of the baseline gait
      float x = pnt[0] * xVel/cs + pnt[1] * yVel/cs;
      float y = pnt[0] * yVel/cs + pnt[1] * xVel/cs;
      //std::cout << "x: " << x << ", y: " << y << ", xVel: " << xVel << ", yVel: " << yVel << ", contact speed: " << cs << std::endl;

      Eigen::Vector3f pntx(
          x,
          y,
          -m_zOffset - pnt[2]
          );
      pntx += m_legOrigins[i];
      //std::cout << "x: " << x << " ,y: " << y << " ,x2: " << pntx[0] << ", y2: " << pntx[1] << std::endl;

#if 1
      // Blend from initial positions into required ones.
      if(m_haveInitialPose && m_blendAlpha < 1.0) {
        pntx = m_initialFootLocations[i] * (1.0 - m_blendAlpha) + pntx * m_blendAlpha;
      }
#endif

      pose.SetLegPosition(i,pntx);
    }

    if(m_haveInitialPose && m_blendAlpha < 1.0) {
      m_blendAlpha += timeStep/m_blendTime;
    }

    return Pose2JointAngles(pose,nextJointAngles);
  }

}}
