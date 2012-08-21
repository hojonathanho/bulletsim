#pragma once
#include <openrave/openrave.h>
#include <vector>
void getArmKinInfo(const OpenRAVE::RobotBasePtr& robot, const OpenRAVE::RobotBase::ManipulatorPtr manip, std::vector<OpenRAVE::KinBody::LinkPtr>& armLinks, std::vector<OpenRAVE::KinBody::JointPtr>& armJoints, std::vector<int>& chainDepthOfBodies);
