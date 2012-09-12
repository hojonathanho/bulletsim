#include "robots/ros2rave.h"
#include <boost/foreach.hpp>

vector<int> ros2rave;

void setupDefaultROSRave() {
  int ros2rave_array[] = {6,  7,  8,  9, 10, 11,  0,  1,  2,  3,  4,  5, 12, 13, 14, 26, 29,
         27, 28, 31, 30, 32, 33, 37, 34, -1, -1, -1, 17, 15, 16, 19, 18, 20,
  		  21, 25, 22, -1, -1, -1};
  ros2rave.assign(ros2rave_array, ros2rave_array + sizeof(ros2rave_array)/sizeof(int));
}

void ROSRaveReset(const OpenRAVE::RobotBasePtr& robot, const sensor_msgs::JointState& msg) {
  // ros2rave = [pr2.GetJointIndex(name) for name in rosnames]
  BOOST_FOREACH(const string& name, msg.name) {
    ros2rave.push_back(robot->GetJointIndex(name));
  }
}

static bool setupDone = false;
void setupROSRave(const OpenRAVE::RobotBasePtr& robot, const sensor_msgs::JointState& msg) {
  if (!setupDone) {
    ROSRaveReset(robot, msg);
    setupDone = true;
  }
}


ValuesInds getValuesInds(const vector<double>& rosjoints) {
  ValuesInds out;
  for (int iros=0; iros<ros2rave.size(); ++iros) {
    if (ros2rave[iros] != -1)  {
      int irave = ros2rave[iros];
      out.first.push_back(rosjoints[iros]);
      out.second.push_back(irave);
    }
  }
    return out;
}
