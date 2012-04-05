#include "simulation/simplescene.h"
#include "robots/pr2.h"
#include <openrave/iksolver.h>
#include <iostream>
#include <vector>
#include <omp.h>
using namespace std;
using namespace OpenRAVE;


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
  PR2Manager pr2m(scene);
  OpenRAVE::Transform tf = pr2m.pr2Left->manip->GetEndEffectorTransform();
  vector< vector<dReal> > vsolution;

  // vector<double> freejoints;
  // OpenRAVE::IkSolverBasePtr pr2solver =  ik_pr2_leftarm::CreateIkSolver(scene.rave->env, freejoints);
  cout << omp_get_max_threads() << " threads" << endl;


  double tStart;

  tStart = timeOfDay();
  vector<RobotBase::ManipulatorPtr> manips;
  for (int i=0; i < omp_get_max_threads(); i++) {
    EnvironmentBasePtr pcloneenv = scene.rave->env->CloneSelf(Clone_Bodies|Clone_Simulation);
    vector<RobotBasePtr> robots;
    pcloneenv->GetRobots(robots);
    RobotBasePtr pr2copy = robots[0];
    RobotBase::ManipulatorPtr manipcopy = pr2copy->GetManipulators()[7];
    manips.push_back(manipcopy);
  }  
  cout << "copying env took " << timeOfDay() - tStart << " seconds" << endl;
  
  tStart = timeOfDay();    
  #pragma omp parallel for
  for (int i=0; i < 1000; i++) {
    manips[omp_get_thread_num()]->FindIKSolutions(IkParameterization(tf), vsolution, IKFO_IgnoreSelfCollisions | IKFO_IgnoreEndEffectorCollisions);
    if (i % 100 == 0) cout << "iteration " << i << endl;
  }


  cout << "total time: " << timeOfDay() - tStart << endl;
}
