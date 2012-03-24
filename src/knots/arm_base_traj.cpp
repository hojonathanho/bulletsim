#include "arm_base_traj.h"
#include "utils_python.h"
#include "utils/interpolation.h"
#include "simulation/util.h"
using namespace OpenRAVE;
using namespace std;
using namespace Eigen;

vector<btTransform> findBasePoses(const vector<btTransform>& leftPoses, const vector<double>& curJoints) {
  int nPoses = leftPoses.size();
  int dims[2] = {nPoses, 7};
  npMatrixf poses(dims);
  for (int i=0; i < nPoses; i++) {
    btQuaternion quat = leftPoses[i].getRotation();
    btVector3 pos = leftPoses[i].getOrigin();
    poses[i][0] = quat.x();
    poses[i][1] = quat.y();
    poses[i][2] = quat.z();
    poses[i][3] = quat.w();
    poses[i][4] = pos.x();
    poses[i][5] = pos.y();
    poses[i][6] = pos.z();
  }

  int dims1[1] = {curJoints.size()};
  npVectord npCurJoints(dims1);
  for (int i=0; i < dims1[0]; i++)  npCurJoints[i] = curJoints[i];

  py::object reachability = py::import("knot_tying.reachability");
  py::object pyResult = reachability.attr("get_base_positions")(toObject(poses),toObject(npCurJoints));
  npMatrixf result(pyResult.ptr());
  
  vector<btTransform> out;
  for (int i=0; i < nPoses; i++) {
    btTransform tf;
    tf.setOrigin(btVector3(result[i][0],result[i][1],0));
    tf.setRotation(btQuaternion(0,0,0,1));
    out.push_back(tf);
  }

  return out;
}

const int DOWNSAMPLE_TRAJ = 100;
const int NOFFSETS = 12;
const float OFFSETS[NOFFSETS][2] = {
  {0,   0},
  {.1,  0},
  {-.1, 0},
  {0,   .1},
  {0,   .2},
  {0,   .3},
  {0,   .4},
  {0,   -.1},
  {0,   -.2},
  {0,   -.3},
  {0,   -.4},
  {0, 20}

};


const float INFEAS_COST = 100;
vector<btTransform> findBasePoses1(OpenRAVE::RobotBase::ManipulatorPtr manip, const vector<btTransform>& leftPoses) {
  int nPosesOrig = leftPoses.size();

  vector<btTransform> downsampledPoses = decimate(leftPoses, 100);
  int nPoses = downsampledPoses.size();
  int nOffsets = NOFFSETS;

  int dims0[3] = {nPoses, nOffsets, nOffsets};
  numpy_boost<float, 3> edgeCost_nkk(dims0);
  int dims1[2] = {nPoses, nOffsets};
  numpy_boost<float, 2> nodeCost_nk(dims1);


  for (int n=0; n < nPoses; n++) {
    for (int k=0; k < nOffsets; k++) {
      OpenRAVE::Transform tf = util::toRaveTransform(downsampledPoses[n]);
      tf.trans[0] -= OFFSETS[k][0];
      tf.trans[1] -= OFFSETS[k][1];
      vector< vector<double> > solns;
      manip->FindIKSolutions(IkParameterization(tf), solns, IKFO_IgnoreSelfCollisions | IKFO_IgnoreEndEffectorCollisions);
      nodeCost_nk[n][k] = (solns.size() == 0) ? INFEAS_COST : 0;
      for (int k1 = 0; k1 < nOffsets; k1++)
	edgeCost_nkk[n][k][k1] = fabs(k - k1);
    }
  }

  for (int k=1; k < nOffsets; k++) nodeCost_nk[0][k] = 100*INFEAS_COST;

  py::object reachability = py::import("knot_tying.reachability");
  py::object pyresult = reachability.attr("shortest_path")(toObject(nodeCost_nk), toObject(edgeCost_nkk));
  py::object pypath = pyresult.attr("__getitem__")(0);


  numpy_boost<int,1> path(pypath.ptr());
  cout << path.shape()[0] << endl;

  Eigen::MatrixXf downsampledBaseXY(nPoses, 2);
  for (int i=0; i < path.shape()[0]; i++) {
    downsampledBaseXY(i,0) = OFFSETS[path[i]][0];
    downsampledBaseXY(i,1) = OFFSETS[path[i]][1];
  }

  Eigen::MatrixXf baseXY = interp2d(VectorXf::LinSpaced(nPosesOrig,0,1), VectorXf::LinSpaced(nPoses,0,1), downsampledBaseXY);

  vector<btTransform> basePoses(baseXY.rows());
  for (int i=0; i < basePoses.size(); i++) {
    basePoses[i].setIdentity();
    basePoses[i].setOrigin(btVector3(baseXY(i,0), baseXY(i,1),0));
  }
  return basePoses;

}
