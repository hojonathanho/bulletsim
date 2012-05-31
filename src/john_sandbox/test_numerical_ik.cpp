#include "simulation/simplescene.h"
#include "robots/pr2.h"
#include <openrave/iksolver.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
using namespace std;
using namespace OpenRAVE;
using namespace Eigen;

static const int POSITION_JAC_DIMS[2] = {3,7};
static const int ROTATION_JAC_DIMS[2] = {4,7};

MatrixXd toEigenMatrix(boost::multi_array<dReal, 2>& in) {
  Map<MatrixXd> out(in.data(),in.shape()[0], in.shape()[1]);
  return out;
}

MatrixXd toEigenMatrix(vector<dReal>& in) {
  Map<MatrixXd> out(in.data(), 1, in.size());
  return out;
}

MatrixXd toEigenMatrix4(RaveVector<dReal>& in) {
  MatrixXd out(1,4);
  out(0,0) = in.x;
  out(0,1) = in.y;
  out(0,2) = in.z;
  out(0,3) = in.w;
  return out;
}

MatrixXd toEigenMatrix3(RaveVector<dReal>& in) {
  MatrixXd out(1,3);
  out(0,0) = in.x;
  out(0,1) = in.y;
  out(0,2) = in.z;
  return out;
}

vector<double> toVector(const MatrixXd& in) {
  vector<double> out;
  out.assign(in.data(), in.data() + in.size());
  return out;
}

void moveByJt(RobotBase::ManipulatorPtr manip, OpenRAVE::Transform target, int nIter) {
  manip->GetRobot()->SetActiveDOFs(manip->GetArmIndices());

  for (int i=0; i < nIter; i++) {
  
    static boost::multi_array<dReal, 2> J_pos(boost::extents[3][7]);
    static boost::multi_array<dReal, 2> J_rot(boost::extents[4][7]);
    manip->CalculateJacobian(J_pos);
    manip->CalculateRotationJacobian(J_rot);

  
    OpenRAVE::Transform current = manip->GetTransform();
  
    MatrixXd deltaPos = (toEigenMatrix3(target.trans) - toEigenMatrix3(current.trans)) * toEigenMatrix(J_pos);
    MatrixXd deltaRot = (toEigenMatrix4(target.rot) - toEigenMatrix4(current.rot)) * toEigenMatrix(J_rot);
  
    vector<double> curJoints;
    manip->GetRobot()->GetActiveDOFValues(curJoints); 
  
    MatrixXd a = toEigenMatrix(curJoints);

    vector<double> newJoints = toVector(toEigenMatrix(curJoints) + .1*deltaPos + .01*deltaRot);
    manip->GetRobot()->SetActiveDOFValues(newJoints);
    // then you need to updateBullet
  }
  
}

double timeOfDay() {
  timeval tim;
  gettimeofday(&tim, NULL);
  return tim.tv_sec+(tim.tv_usec/1000000.0);
}

// namespace OpenRAVE{
// namespace ik_pr2_leftarm {
//   extern OpenRAVE::IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr, const std::vector<dReal>&vfreeinc);
// }
// }

int main(int argc, char* argv[]) {
  Scene scene;
  scene.startViewer();
  PR2Manager pr2m(scene);
  OpenRAVE::Transform current = pr2m.pr2Left->manip->GetEndEffectorTransform();
  vector< vector<dReal> > vsolution;

  // vector<double> freejoints;
  // OpenRAVE::IkSolverBasePtr pr2solver =  ik_pr2_leftarm::CreateIkSolver(scene.rave->env, freejoints);


  RobotBase::ManipulatorPtr manip = pr2m.pr2Left->manip;
  PlotAxes::Ptr axes(new PlotAxes());
  scene.env->add(axes);
  
  OpenRAVE::Transform target = current;
  while (true) {
//	  moveByJt(manip, target, 1);
	  pr2m.pr2->updateBullet();
	  btTransform btarg = util::toBtTransform(target);
	  axes->setup(btarg, .1);
	  scene.step(.01);
  }

}
