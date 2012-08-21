#include "arm_base_traj.h"
#include "utils_python.h"
#include "utils/interpolation.h"
#include "simulation/util.h"
#include "knots.h"
using namespace OpenRAVE;
using namespace std;
using namespace Eigen;

static inline float f(py::object o) {return py::extract<float>(o);}

vector<btTransform> findBasePoses(const vector<btTransform>& leftPoses, const vector<double>& curJoints) {
  int nPoses = leftPoses.size();

  py::object poses = NP.attr("empty")(py::make_tuple(nPoses, 7));
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


  py::object npCurJoints = NP.attr("empty")(curJoints.size());
  for (int i=0; i < curJoints.size(); i++)  npCurJoints[i] = curJoints[i];

  py::object reachability = py::import("knot_tying.reachability");
  py::object result = reachability.attr("get_base_positions")(poses,npCurJoints);
  
  vector<btTransform> out;
  for (int i=0; i < nPoses; i++) {
    btTransform tf;
    tf.setOrigin(btVector3(f(result[i][0]),f(result[i][1]),0));
    tf.setRotation(btQuaternion(0,0,0,1));
    out.push_back(tf);
  }

  return out;
}

const int DOWNSAMPLE_TRAJ = 100;
const float INFEAS_COST = 100;

vector<btTransform> findBasePoses1(OpenRAVE::RobotBase::ManipulatorPtr manip, const vector<btTransform>& leftPoses) {
  cout << manip->GetName() << endl;
  int nPosesOrig = leftPoses.size();

  vector<btTransform> downsampledPoses = decimate(leftPoses, 20);

  const int N_DX = 1;
  const float DX[N_DX] = {0};//{-.2, -.1, 0, .1, .2};
  const int N_DY = 9;
  const float DY[N_DY] = {-.4, -.3, -.2, -.1, 0, .1, .2, .3, .4};

  int NOFFSETS = N_DX * N_DY;
  float OFFSETS[NOFFSETS][2];

  int count=0;
  for (int i=0; i < N_DX; i++) {
    for (int j=0; j < N_DY; j++) {
      OFFSETS[count][0] = DX[i];
      OFFSETS[count][1] = DY[j];
      count++;
    }
  }
  
  int nPoses = downsampledPoses.size();
  int nOffsets = NOFFSETS;

  py::object numpy = py::import("numpy");
  py::object edgeCost_nkk = numpy.attr("zeros")(py::make_tuple(nPoses, nOffsets, nOffsets));
  py::object nodeCost_nk = numpy.attr("zeros")(py::make_tuple(nPoses, nOffsets));


  for (int n=0; n < nPoses; n++) {
    for (int k=0; k < nOffsets; k++) {
      OpenRAVE::Transform tf = util::toRaveTransform(downsampledPoses[n]);
      tf.trans[0] -= OFFSETS[k][0];
      tf.trans[1] -= OFFSETS[k][1];
      vector< vector<double> > solns;
      manip->FindIKSolutions(IkParameterization(tf), solns, IKFO_IgnoreSelfCollisions | IKFO_IgnoreEndEffectorCollisions);
      nodeCost_nk[n][k] = (solns.size() == 0) ? INFEAS_COST : -sqrtf(solns.size());
      for (int k1 = 0; k1 < nOffsets; k1++)
	edgeCost_nkk[n][k][k1] = fabs(k - k1);
    }
  }

  // for (int k=1; k < nOffsets; k++) nodeCost_nk[0][k] = 100*INFEAS_COST;

  py::object reachability = py::import("knot_tying.reachability");
  py::object pyresult = reachability.attr("shortest_path")(nodeCost_nk, edgeCost_nkk);
  py::object pypath = pyresult[0]; // bracket operator does __getitem__!


  Eigen::MatrixXf downsampledBaseXY(nPoses, 2);
  for (int i=0; i < nPoses; i++) {
    downsampledBaseXY(i,0) = OFFSETS[py::extract<int>(pypath[i])][0];
    downsampledBaseXY(i,1) = OFFSETS[py::extract<int>(pypath[i])][1];
  }

  Eigen::MatrixXf baseXY = interp2f(VectorXf::LinSpaced(nPosesOrig,0,1), VectorXf::LinSpaced(nPoses,0,1), downsampledBaseXY);

  vector<btTransform> basePoses(baseXY.rows());
  for (int i=0; i < basePoses.size(); i++) {
    basePoses[i].setIdentity();
    basePoses[i].setOrigin(btVector3(baseXY(i,0), baseXY(i,1),0));
  }
  return basePoses;

}

extern py::object toNumpy1(const vector<RobotAndRopeState>& rars);

vector<btTransform> findBasePoses2(const vector<RobotAndRopeState>& states, vector<double> curJoints) {
  int nPoses = states.size();

  py::object npCurJoints = NP.attr("empty")(curJoints.size());
  for (int i=0; i < curJoints.size(); i++)  npCurJoints[i] = curJoints[i];

  py::object reachability = py::import("knot_tying.reachability2");
  py::object pystates = toNumpy1(states);
  reachability.attr("init")(0);
  py::object result = reachability.attr("get_base_positions_bimanual_resampled")(pystates, npCurJoints); 
 
  vector<btTransform> out;
  for (int i=0; i < nPoses; i++) {
    btTransform tf;
    tf.setOrigin(btVector3(f(result[i][0]),f(result[i][1]),0));
    tf.setRotation(btQuaternion(0,0,0,1));
    out.push_back(tf);
  }

  return out;
}
