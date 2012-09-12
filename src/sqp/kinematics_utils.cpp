#include "kinematics_utils.h"
#include <boost/foreach.hpp>
using namespace OpenRAVE;
using namespace std;

void getArmKinInfo(const RobotBasePtr& robot, const RobotBase::ManipulatorPtr manip, std::vector<KinBody::LinkPtr>& armLinks, std::vector<KinBody::JointPtr>& armJoints, std::vector<int>& chainDepthOfBodies) {
  int rootLinkInd               = robot->GetLink("torso_lift_link")->GetIndex();
  BOOST_FOREACH(int ind, manip->GetArmIndices()) armJoints.push_back(robot->GetJointFromDOFIndex(ind));
  KinBody::JointPtr& firstJoint = armJoints[0];

  BOOST_FOREACH(KinBody::LinkPtr link, robot->GetLinks()) {
    vector<KinBody::JointPtr> jointChain;
    robot->GetChain(rootLinkInd, link->GetIndex(), jointChain);
    if (link->GetGeometries().size() && count(jointChain.begin(), jointChain.end(), firstJoint)) armLinks.push_back(link);
  }

  int nLinks                    = armLinks.size();

  chainDepthOfBodies            = vector<int>(nLinks,0);
  for (int iLink                = 0; iLink < armLinks.size(); ++iLink) {
    int linkInd                 = armLinks[iLink]->GetIndex();
    vector<KinBody::JointPtr> jointChain;
    robot->GetChain(rootLinkInd, armLinks[iLink]->GetIndex(), jointChain);
    BOOST_FOREACH(KinBody::JointPtr& joint0, armJoints) {
      BOOST_FOREACH(KinBody::JointPtr& joint1, jointChain) {
        if (joint0 == joint1) chainDepthOfBodies[iLink]++;
      }
    }
  }
}
