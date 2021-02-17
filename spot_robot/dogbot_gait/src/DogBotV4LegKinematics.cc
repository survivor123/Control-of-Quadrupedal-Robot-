
/* Contains two main classes: DogBotV4LegKinematicsC, which handles single-leg calculations,
 * and DogBotV4KinematicsC for the whole quadruped robot
 */

#include "ReactRobotics/LineABC2d.hh"
#include "ReactRobotics/DogBotV4LegKinematics.hh"

#include <math.h>
#include <assert.h>
#include <iostream>

#define DODEBUG 0
#if DODEBUG
#define ONDEBUG(x) x
#else
#define ONDEBUG(x)
#endif

namespace ReactRoboticsN
{

  static float Sqr(float val)
  { return val * val; }

  DogBotV4LegKinematicsC::DogBotV4LegKinematicsC()
  {
    Init();
    ONDEBUG(std::cout << " Name:" << m_name << "  Origin:" << m_legOrigin[0] << " " << m_legOrigin[1] << " " << m_legOrigin[2] << " " << std::endl);
    ONDEBUG(std::cout << "  Leg: L1=" << m_l1 << " L2=" << m_l2 << std::endl);
  }

  //! Construct from a json object
  DogBotV4LegKinematicsC::DogBotV4LegKinematicsC(const Json::Value &value)
  {
    ConfigureFromJSON(value);
  }

  //! Configure from JSON
  bool DogBotV4LegKinematicsC::ConfigureFromJSON(const Json::Value &value)
  {
    m_name = value.get("name","").asString();
    m_l0 = value.get("l0",m_l0).asFloat();
    m_l1 = value.get("l1",m_l1).asFloat();
    m_l2 = value.get("l2",m_l2).asFloat();

    m_footDrop = value.get("FootDrop",m_footDrop).asFloat();
    m_footSphereRadius = value.get("FootSphereRadius",m_footSphereRadius).asFloat();

    m_jointDirections[0] = value.get("dirRoll",m_jointDirections[0]).asFloat();
    m_jointDirections[1] = value.get("dirPitch",m_jointDirections[1]).asFloat();
    m_jointDirections[2] = value.get("dirKnee",m_jointDirections[2]).asFloat();


    ONDEBUG(std::cout << " Name:" << m_name << "  Origin:" << m_legOrigin[0] << " " << m_legOrigin[1] << " " << m_legOrigin[2] << " " << std::endl);
    ONDEBUG(std::cout << "  Leg: L1=" << m_l1 << " L2=" << m_l2 << " " << std::endl);
    Init();
    return true;
  }

  void DogBotV4LegKinematicsC::Init()
  {
    m_minExtension = sqrt(Sqr(m_l0) + Sqr(m_footSphereRadius + 0.06));
    //m_maxExtension = m_l1 + m_l2;
    m_maxExtension = sqrt(Sqr(m_l1 + m_l2) + Sqr(m_l0));
    //std::cout << "Min extension " << m_minExtension << " to max " << m_maxExtension << std::endl;
  }

  //! Get the servo configuration as JSON
  Json::Value DogBotV4LegKinematicsC::ConfigAsJSON() const
  {
    Json::Value ret;

    ret["name"] = m_name;
    ret["l0"] = m_l0;
    ret["l1"] = m_l1;
    ret["l2"] = m_l2;

    ret["dirRoll"] = m_jointDirections[0];
    ret["dirPitch"] = m_jointDirections[1];
    ret["dirKnee"] = m_jointDirections[2];

    ret["FootDrop"] = m_footDrop;
    ret["FootSphereRadius"] = m_footSphereRadius;

    return ret;
  }

  //! Clip foot position to nearest valid location
  // Returns true if within bound and false if clipped.
  bool DogBotV4LegKinematicsC::ClipPosition(Eigen::Vector3f &at) const
  {
    bool ret = true;
    Eigen::Vector3f target = at - m_legOrigin;

    // FIXME:- Do a better job of selecting allowable positions.
    const float maxHeight = -0.06;
    if(target[2] > maxHeight)
      target[2] = maxHeight;

    // Set target to closest we can actually reach.
    float ext = target.norm();
    if(ext < m_minExtension) {
      //std::cerr << "Limiting min extension from " << ext << " to " << m_minExtension << " ." << std::endl;
      if(ext != 0)
        target *= m_minExtension / ext;
      else
        target = Eigen::Vector3f(0,0,-m_minExtension);
      ret = false;
    }
    if(ext > m_maxExtension) {
      //std::cerr << "Limiting max extension from " << ext << " to " << m_maxExtension << " on " << m_name << " ." << std::endl;
      ext = m_maxExtension;
      if(ext != 0)
        target *= m_maxExtension / ext;
      ret = false;
    }
    at = target + m_legOrigin;
    return ret;
  }


  //! Inverse kinematics for the leg
  //! Compute joint angles needed to get to a 3d position in a leg coordinate system
  //! Return true if position is reachable
  bool DogBotV4LegKinematicsC::Inverse(const Eigen::Vector3f &at,Eigen::Vector3f &angles) const
  {
    bool ret = true;
    Eigen::Vector3f target = at;
    ClipPosition(target);
    target -= m_legOrigin;

    float x = target[0];
    float y = target[1];
    float z = -target[2];

    {
      float l2 = Sqr(z) + Sqr(x);
      float l = sqrt(l2);
      float a2 = l2 - Sqr(m_l0);
      if(a2 < 0) {
        //std::cerr << "No roll solution " << a2 << std::endl;
        ret = false;
        a2 = 0;
        //return false;
      }
      float nz = sqrt(a2);
      float a1 = (atan2(z,-x) - acos(m_l0/l));

#if 0
      float ta = atan2(x,z);
      z = sin(ta) * x + cos(ta) * z ;
#endif
      angles[0] = a1;
      z = nz;
    }

    {
      float l2 = Sqr(z) + Sqr(y);
      double ac1 = (l2 + Sqr(m_l1) - Sqr(m_l2))/(2 * sqrt(l2) * m_l1);
      if(ac1 < -1 || ac1 > 1) {
        //std::cerr << "No pitch solution. " << ac1 << " x:" << x << " y:" << y << " z:" << z << std::endl;
        ret = false;
        // Fudge it...
        if(ac1 > 1)
          ac1 = 1.0;
        if(ac1 < -1)
          ac1 = -1.0;
      }

      float a1 = atan2(-y,z);
      angles[1] = -1 *(a1 + acos(ac1));

      double ac2 = (Sqr(m_l2) + Sqr(m_l1) -l2)/(2 * m_l2 * m_l1);
      if(ac2 < -1 || ac2 > 1) {
        //std::cerr << "No knee solution. " << ac2 << " " << std::endl;
        // Just try our best.
        ret = false;
        if(ac2 < -1)
          ac2 = -1;
        if(ac2 > 1)
          ac2 = 1;
        //return false;
      }

      float kneeTarget = M_PI - acos(ac2);

      angles[2] = kneeTarget;
    }

    for(int i = 0;i < 3;i++)
      angles[i] *= m_jointDirections[i];

    // Bring angles into a sensible range.
    if(angles[2] > M_PI)
      angles[2] -= M_PI*2;
    if(angles[2] < -M_PI)
      angles[2] += M_PI*2;

    return ret;
  }

  //! Forward kinematics for the leg
  //! Compute the position of the foot relative to the top of the leg from the joint angles.
  bool DogBotV4LegKinematicsC::Forward(const Eigen::Vector3f &anglesIn,Eigen::Vector3f &at) const
  {
    Eigen::Vector3f angles;

    for(int i = 0;i < 3;i++)
      angles[i] = anglesIn[i] * m_jointDirections[i];

    //std::cerr << "FV PSI=" << angles[2] << " Pitch=" << angles[1] << std::endl;

    float y = m_l1 * sin(angles[1]) + m_l2 * sin(angles[1] + angles[2]);
    float z = m_l1 * cos(angles[1]) + m_l2 * cos(angles[1] + angles[2]);

    //std::cerr << "FV Y= " << y << " Z=" << z << std::endl;

    //float xr = 0;

    float s = sin(angles[0]);
    float c = cos(angles[0]);

    at[0] =  s * (z) - c * m_l0;
    at[1] =  y;
    at[2] =  -(c * (z) + s * m_l0);

    at += m_legOrigin;
    //for(int i = 0;i < 3;i++) at[i] += m_legOrigin[i];

    return true;
  }

  //! Compute an estimate of the force on a foot and where it is given some angles and torques
  bool DogBotV4LegKinematicsC::ComputeFootForce(
      const Eigen::Vector3f &jointAngles,
      const Eigen::Vector3f &jointVelocity,
      const Eigen::Vector3f &rawTorques,
      Eigen::Vector3f &footAt,
      Eigen::Vector3f &footVelocity,
      Eigen::Vector3f &force
      ) const
  {
    Eigen::Vector3f torque = rawTorques;

    Eigen::Vector3f axisRoll(0,-1,0);
    Eigen::Vector3f axisPitch(-1,0,0);
    Eigen::Vector3f axisKnee(-1,0,0);

    // Compute current position

    double angleRoll = jointAngles[0] * JointDirection(0);
    double anglePitch = jointAngles[1] * JointDirection(1);
    double angleKnee = jointAngles[2] * JointDirection(2);

    double velocityRoll = jointVelocity[0] * JointDirection(0);
    double velocityPitch = jointVelocity[1] * JointDirection(1);
    double velocityKnee = jointVelocity[2] * JointDirection(2);

    for(int i = 0;i < 3;i++)
      torque[i] *= JointDirection(i);

    // Theta = Knee Servo
    // Psi = Knee joint


    //std::cerr << "CFF PSI=" << psi << " Pitch=" << anglePitch << std::endl;

    float y = m_l1 * sin(anglePitch) + m_l2 * sin(angleKnee + anglePitch);
    float z = m_l1 * cos(anglePitch) + m_l2 * cos(angleKnee + anglePitch);

    //std::cerr << "CFF Y= " << y << " Z=" << z << std::endl;
    // kneeOffset, lowerLegOffset and footOffset are in the hip coordinate system

    Eigen::Vector2f upperLegOffset(
        m_l1 * sin(anglePitch),
        m_l1 * cos(anglePitch)
        );

    Eigen::Vector2f lowerLegOffset(
        m_l2 * sin(angleKnee + anglePitch),
        m_l2 * cos(angleKnee + anglePitch)
        );

    Eigen::Vector2f footOffset = upperLegOffset + lowerLegOffset;

    float footOffsetNorm2 = footOffset.squaredNorm();

    float kneeTorque = -torque[2];
    float pitchTorque = torque[1];

    // Pitch force
    Eigen::Vector2f pf = Perpendicular(footOffset) * pitchTorque / footOffsetNorm2;

    // Knee force
    Eigen::Vector2f kf = Perpendicular(lowerLegOffset) * kneeTorque / lowerLegOffset.squaredNorm();

    LineABC2dC line1 = LineABC2dC::CreateFromNormalAndPoint(pf,pf);
    LineABC2dC line2 = LineABC2dC::CreateFromNormalAndPoint(kf,kf);

    Eigen::Vector2f hipf;
    line1.Intersection(line2,hipf);

#if 0
    m_log->info("Torque P:{:+2.2f}  K:{:2.2f}  Pf: {:+2.2f} {:+2.2f}  Kf: {:+2.2f} {:+2.2f}  Hip force  {:+2.2f} {:+2.2f} ",
                pitchTorque,kneeTorque,
                pf[0],pf[1],
                kf[0],kf[1],
                hipf[0],hipf[1]
                             );
#endif

    auto rollRot = Eigen::AngleAxisf(angleRoll,axisRoll);

    Eigen::Vector3f legForce = rollRot * Eigen::Vector3f(0,hipf[0],hipf[1]);

    // Location of centre of knee and pitch servos
    //Eigen::Vector3f hipCentreAt(sin(angleRoll) * zoff,0,cos(angleRoll) * zoff);

    // footAt_fc is in leg coordinates
    const Eigen::Vector3f footAt_fc(
        -sin(angleRoll) * ( z), //cos(angles[0]) * xr +
        y,
        -cos(angleRoll) * ( z) //sin(angles[0]) * xr +
        );

    // Compute the force from roll on the foot.
    Eigen::Vector3f rollTangent = footAt_fc.cross(axisRoll);

    Eigen::Vector3f rollForce = rollTangent * torque[0] /footAt_fc.squaredNorm();

    // Compute total force in leg coordinates.
    force = rollForce + legForce;

    // Compute the location of the foot in robot coordinates.
    footAt = footAt_fc ;// + m_legOrigin

#if 0
    m_log->info("Torques: {:+2.2f} {:+2.2f} {:+2.2f}  Virt: {:+2.2f} {:+2.2f} {:+2.2f} ({:+1.2})   Leg: {:+2.2f} {:+2.2f} {:+2.2f} ",
                torque[0],torque[1],torque[2],
                torque[0],pitchTorque,kneeTorque,
                ratio,
                force[0],force[1],force[2]);
#endif

    // Compute velocities.

    Eigen::Vector2f kneeVel2d =  Perpendicular(lowerLegOffset) * velocityKnee ;
    //std::cerr << "Foot " << footOffset[0] << " " << footOffset[1] <<  " KneeVel:" << kneeVel2d[0] << " " << kneeVel2d[1] << std::endl;
    Eigen::Vector2f pitchVel2d = Perpendicular(footOffset) * velocityPitch + kneeVel2d;

    footVelocity = rollRot * Eigen::Vector3f(0,-pitchVel2d[0],pitchVel2d[1]) + rollTangent * velocityRoll;
    return true;
  }

  //! Compute the maximum stride length at a given z offset.
  float DogBotV4LegKinematicsC::StrideLength(float zoffset) const
  {
    float r = m_l1 + m_l2;

    float rh = zoffset ;
    float sqrs = r*r - rh*rh;
    if(sqrs <= 0) {
      return 0;
    }
    return 2*sqrt(sqrs);
  }


  // ---------------------------------------------------------------------

  DogBotV4KinematicsC::DogBotV4KinematicsC()
  {
    m_legKinimatics[0].SetOrigin(Eigen::Vector3f(-m_bodyWidth/2.0,m_bodyLength/2.0,0));
    m_legKinimatics[0].SetJointDirections(Eigen::Vector3f(-1.0,-1.0,-1.0));
    m_legKinimatics[0].SetName("front_left");
    m_legKinimatics[1].SetJointDirections(Eigen::Vector3f(-1.0,1.0,1.0));
    m_legKinimatics[1].SetOrigin(Eigen::Vector3f(m_bodyWidth/2.0,m_bodyLength/2.0,0));
    m_legKinimatics[1].SetName("front_right");
    m_legKinimatics[1].InvertL0();
    m_legKinimatics[2].SetOrigin(Eigen::Vector3f(-m_bodyWidth/2.0,-m_bodyLength/2.0,0));
    m_legKinimatics[2].SetJointDirections(Eigen::Vector3f(1.0,-1.0,-1.0));
    m_legKinimatics[2].SetName("back_left");
    m_legKinimatics[3].SetOrigin(Eigen::Vector3f(m_bodyWidth/2.0,-m_bodyLength/2.0,0));
    m_legKinimatics[3].SetJointDirections(Eigen::Vector3f(1.0,1.0,1.0));
    m_legKinimatics[3].SetName("back_right");
    m_legKinimatics[3].InvertL0();
  }

  //! Convert feet position to joint angles, in the order given in 'JointNames'
  bool DogBotV4KinematicsC::FeetPositions2JointAngles(
      const std::array<Eigen::Vector3f,4> &positions,
      Eigen::VectorXf &angles
      )
  {
    bool ok = true;
    if(angles.size() != 12)
      angles = Eigen::VectorXf(12);
    auto pp = positions.begin();
    for(int i = 0;i < 4;i++,pp++) {
      Eigen::Vector3f jangles;
      if(!m_legKinimatics[i].Inverse(positions[i],jangles))
        ok = false;
      for(int j = 0;j < 3;j++)
        angles[i*3 + j] = jangles[j];
    }
    return ok;
  }

  //! Convert joint angles to feet position
  // Positions must be an array of at least 4 Vector3f's
  bool DogBotV4KinematicsC::JointAngles2FeetPositions(const Eigen::VectorXf &angles,std::array<Eigen::Vector3f,4> &positions)
  {
    bool ok = true;

    for(int i = 0;i < 4;i++) {
      Eigen::Vector3f jangles;
      jangles[0] = angles[i*3 + 0];
      jangles[1] = angles[i*3 + 1];
      jangles[2] = angles[i*3 + 2];
      if(!m_legKinimatics[i].Forward(jangles,positions[i]))
        ok = false;
    }

    return ok;
  }

  //! Clip foot positions to nearest valid location
  // Returns true if within bound and false if clipped.
  bool DogBotV4KinematicsC::ClipPositions(std::array<Eigen::Vector3f,4> &feetPositions) const
  {
    bool ok = true;
    for(int i = 0;i < 4;i++) {
      if(!m_legKinimatics[i].ClipPosition(feetPositions[i]))
        ok = false;
    }
    return ok;
  }


  //! Access a list of joint names
  std::vector<std::string> DogBotV4KinematicsC::JointNames()
  {
    std::vector<std::string> ret;
    const char **at = m_jointNames;
    for(;(*at) != nullptr;at++)
      ret.push_back(std::string(*at));
    return ret;
  }

  const char *DogBotV4KinematicsC::m_jointNames[] = {
        "front_left_roll_joint",
        "front_left_pitch_joint",
        "front_left_knee_joint",
        "front_right_roll_joint",
        "front_right_pitch_joint",
        "front_right_knee_joint",
        "back_left_roll_joint",
        "back_left_pitch_joint",
        "back_left_knee_joint",
        "back_right_roll_joint",
        "back_right_pitch_joint",
        "back_right_knee_joint",
        nullptr
    };



}

