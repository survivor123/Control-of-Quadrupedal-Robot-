
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <thread>
#include <vector>
#include <functional>
#include <assert.h>
#include <mutex>
#include <eigen3/Eigen/Geometry>

#include "ReactRobotics/GaitController/SplineGaitController.hh"

std::string ReplaceString(const std::string &str,const std::string &oldText,const std::string &newText)
{
  std::string ret = str;
  ret.replace(ret.find(oldText),oldText.size(),newText);
  return ret;
}


std::vector<int> BuildJointMap(
    const std::vector<std::string> &jointNames,
    const sensor_msgs::JointState &jointState
    )
{
  std::vector<int> ret;
  ret.reserve(jointState.name.size());
  for(auto &jn : jointState.name) {
    auto at = std::find(jointNames.begin(),jointNames.end(),jn);
    if(at == jointNames.end()) {
      std::cerr << "Joint " << jn << " not found. " << std::endl;
      throw std::invalid_argument("no joint");
    }
    int offset = at - jointNames.begin();
    ret.push_back(offset);
  }
  return ret;
}

class ControlVectorC
{
public:
  ControlVectorC()
    : m_twist(6)
  {
    // Default to walking forward
    m_twist.fill(0);
    m_twist[1] = 0.25;
  }

  std::mutex m_access;
  Eigen::VectorXf m_twist;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dogbot_gait");
  ros::NodeHandle n;
  ros::NodeHandle nPrivate("~");

  std::string robotns = "dogbot";
  nPrivate.getParam("robot",robotns);

  float loopHz = 100;
  float omega = 6.5;
  nPrivate.getParam("loop_hz",loopHz);
  nPrivate.getParam("omega",omega);

  ros::AsyncSpinner spinner(2);

  sensor_msgs::JointState::ConstPtr lastJointState;

  sleep(1);

  ReactRoboticsN::GaitControllerN::SplineGaitControllerC gaitGenerator;

  gaitGenerator.SetStyle("trot");
  gaitGenerator.SetOmega(omega);

  std::vector<std::string> jointNames = gaitGenerator.JointNames();

  // Setup topics to publish joint control

  std::vector<ros::Publisher> jointControllers;
  jointControllers.reserve(jointNames.size());

  for(auto &jn : jointNames) {
    std::string newName = ReplaceString(jn,"_joint","_position_controller");
    std::string newPath = std::string("/dogbot/") + newName + std::string("/command");
    jointControllers.push_back(n.advertise<std_msgs::Float64>(newPath,2));
  }

  // Listen for direction commands

  ControlVectorC controlData;
  std::string steerTopicName = "/dogbot/cmd_vel";
  ros::Subscriber subSteer = n.subscribe<geometry_msgs::Twist>(
      steerTopicName,1, [&controlData](const geometry_msgs::Twist::ConstPtr &msg) mutable
      {
        // Convert messages into a command and swap the coordinate system so
        // x is forward.
        std::lock_guard<std::mutex> lock(controlData.m_access);
        controlData.m_twist[0] = msg->linear.y;
        controlData.m_twist[1] = msg->linear.x;
        controlData.m_twist[2] = -msg->linear.z;
        controlData.m_twist[3] = msg->angular.y;
        controlData.m_twist[4] = msg->angular.x;
        controlData.m_twist[5] = -msg->angular.z;
      }
   );

  // Listen for joint states and update the control outputs.

  std::string jointStateTopicName = "/dogbot/joint_states";
  std::vector<int> jointStateName2pos;
  double lastTime = -1;

  ros::Subscriber subJoints = n.subscribe<sensor_msgs::JointState>(
      jointStateTopicName, 1,
      [&lastTime,&controlData,&gaitGenerator,&jointControllers,&jointStateName2pos,&jointNames](const sensor_msgs::JointState::ConstPtr &jointState) mutable
        {
          double atTime = jointState->header.stamp.toSec();
          double updateInterval = 0.02;
          if(lastTime > 0) {
            updateInterval = atTime - lastTime;
          }
          lastTime = atTime;

          // Do we need to build a new map for converting the joint orders?
          if(jointStateName2pos.size() != jointState->name.size())
          { jointStateName2pos = BuildJointMap(jointNames,*jointState); }

          // Build a quadruped state.

          Eigen::VectorXf position(jointStateName2pos.size());
          Eigen::VectorXf velocity(jointStateName2pos.size());
          Eigen::VectorXf torque(jointStateName2pos.size());

          for(int i = 0;i < jointStateName2pos.size();i++) {
            int at = jointStateName2pos[i];
            position[at] = jointState->position[i];
            velocity[at] = jointState->velocity[i];
            torque[at] = jointState->effort[i];
          }

          ReactRoboticsN::GaitControllerN::QuadrupedStateC quadrupedState(
              position,
              velocity,
              torque
              );

          // Where do we want to go ?
          Eigen::VectorXf nextJointAngles,controlVector;
          {
            std::lock_guard<std::mutex> lock(controlData.m_access);
            controlVector = controlData.m_twist;
          }

          // Compute new leg positions
          gaitGenerator.Step(updateInterval, quadrupedState, controlVector, nextJointAngles);

          // Publish angles on command topics
          for(int i = 0;i < nextJointAngles.size();i++) {
            std_msgs::Float64 msg;
            msg.data = nextJointAngles[i];
            //std::cout << " Joint " << i << " = " << msg.data << std::endl;
            jointControllers[i].publish(msg);
          }
        });


  // Wait until shutdown signal received
  spinner.start();
  ros::waitForShutdown();

  std::cout << "Done " << std::endl;

  return 0;
}
