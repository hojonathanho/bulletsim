#include "simulation/simplescene.h"
#include "robots/pr2.h"
#include <boost/timer.hpp>

int main() {
  Scene scene;
  PR2Manager pr2m(scene);

  vector<double> dofs = pr2m.pr2->getDOFValues();
  vector<int> indices(dofs.size());
  for (int i = 0; i < indices.size(); ++i) {
    indices[i] = i;
  }

  boost::timer t;
  int N = 100000;
  for (int i = 0; i < N; ++i) {
    pr2m.pr2->setDOFValues(indices, dofs);
  }
  double elapsed = t.elapsed();
  cout << "time elapsed: " << elapsed << " ( / " << N << " = " << elapsed/(double)N << " )" << endl;

  return 0;
}