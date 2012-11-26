#include <nlopt.hpp>
#include <iostream>
using namespace std;

static double cost(const vector<double> &x, vector<double> &g, void *) {
  if (!g.empty()) {
    g[0] = 2*x[0];
    g[1] = 2*x[1];
  }
  return x[0]*x[0] + x[1]*x[1];
}

int main() {
  nlopt::opt opt(nlopt::LD_LBFGS, 2);
  vector<double> lb(2); lb[0] = 0.1; lb[1] = -HUGE_VAL;
  vector<double> ub(2); ub[0] = 1; ub[1] = HUGE_VAL;
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_vector_storage(100000);
  opt.set_min_objective(cost, NULL);

  vector<double> x0(2); x0[0] = 0.5; x0[1] = 0.5;
  double minf;
  opt.optimize(x0, minf);

  cout << "min: " << minf << '\n';
  cout << "x: " << x0[0] << ' ' << x0[1] << endl;

  return 0;
}
