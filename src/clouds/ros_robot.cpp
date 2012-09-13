#include "robots/ros2rave.h"
#include "ros_robot.h"
#include <ros/ros.h>

RobotSync::RobotSync(ros::NodeHandle& nh, RaveRobotObject::Ptr robot) {
  m_robot = robot;
  m_jointSub = nh.subscribe("/joint_states", 5, &RobotSync::jointCB, this);
}


void RobotSync::jointCB(const sensor_msgs::JointState& msg) {
  m_lastMsg = msg;
}

void RobotSync::updateRobot() {
  setupROSRave(m_robot->robot, m_lastMsg);
  ValuesInds vi = getValuesInds(m_lastMsg.position);
  m_robot->setDOFValues(vi.second, vi.first);
}
