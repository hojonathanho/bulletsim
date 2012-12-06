#include "simulation/simplescene.h"
#include "robots/pr2.h"
#include <boost/timer.hpp>
#include "pr2_fk.h"
#include "ophys_common.h"
using namespace Eigen;
using namespace ophys;

int main() {
  Scene scene;
  PR2Manager pr2m(scene);

  vector<double> dofs = pr2m.pr2->getDOFValues();
  vector<int> indices(dofs.size());
  for (int i = 0; i < indices.size(); ++i) {
    indices[i] = i;
  }

  PR2FastFK fk(false);
  // dof limits:   -2.2854 -0.523601  -3.90001  -2.32131    -10000     -2.18    -10000
  //               0.714602 1.3963    0.8      0  10000      0  10000
  Vector7d lb; lb << -2.2853, -0.523600,  -3.90000,  -2.32130 ,   -10000 ,    -2.17  ,  -10000;
  cout << "fast: " << fk.jointToPos(lb).transpose() << endl;
  pr2m.pr2Right->setDOFValues(toStlVec(lb));
  cout << "orig: " << toEigVec(pr2m.pr2Right->getTransform().getOrigin()).transpose() << endl;

  Vector7d ub; ub << 0.714601, 1.3962,    0.7,      0,  10000,      0,  10000;
  cout << "fast: " << fk.jointToPos(ub).transpose() << endl;
  pr2m.pr2Right->setDOFValues(toStlVec(ub));
  cout << "orig: " << toEigVec(pr2m.pr2Right->getTransform().getOrigin()).transpose() << endl;

  cout << "\nafter calibration:\n";
  fk.calibrate(pr2m.pr2Right);
  Vector7d vals = 0.1*lb + 0.9*ub;
  cout << "fast: " << fk.jointToPos(vals).transpose() << endl;
  pr2m.pr2Right->setDOFValues(toStlVec(vals));
  cout << "orig: " << toEigVec(pr2m.pr2Right->getTransform().getOrigin()).transpose() << endl;

  int N = 100000;
  {
    cout << "usual fk speed test:" << endl;
    boost::timer t;
    for (int i = 0; i < N; ++i) {
      pr2m.pr2->setDOFValues(indices, dofs);
    }
    double elapsed = t.elapsed();
    cout << "time elapsed: " << elapsed << " ( / " << N << " = " << elapsed/(double)N << " )" << endl;
  }

  {
    cout << "\n\nfast fk speed test:" << endl;
    boost::timer t;
    Vector3d p;
    for (int i = 0; i < N; ++i) {
      p = fk.jointToPos(vals);
    }
    double elapsed = t.elapsed();
    cout << "time elapsed: " << elapsed << " ( / " << N << " = " << elapsed/(double)N << " )" << endl;
  }

  return 0;
}
