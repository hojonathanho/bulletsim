#include "openrave_joints.h"
// rosnames = np.load("/home/joschu/Data/knot_kinect2/joint_names.npy") 
// (obtained from a joint_states message)
// ros2rave = [pr2.GetJointIndex(name) for name in rosnames]

int ros2rave[]={ 6,  7,  8,  9, 10, 11,  0,  1,  2,  3,  4,  5, 12, 38, 13, 14, 26,
       29, 27, 28, 31, 30, 32, 33, 37, 34, -1, -1, -1, 36, 35, 17, 15, 16,
	     19, 18, 20, 21, 25, 22, -1, -1, -1, 24, 23};
int nros = 45;

ValuesInds getValuesInds(const vector<double>& rosjoints) {
  ValuesInds out;
  for (int iros=0; iros<nros; iros++) {
    if (ros2rave[iros] != -1)  {
      int irave = ros2rave[iros];
      out.first.push_back(rosjoints[iros]);
      out.second.push_back(irave);
    }
  }
    return out;
}
