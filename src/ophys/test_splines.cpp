#include <iostream>
#include "splines.h"
using namespace Eigen;
using namespace std;

int main() {
  VectorXd x(VectorXd::LinSpaced(5, 0, 4));
  VectorXd f(5); f << 6, 7, 8, 9, 10;
  VectorXd df(5); df << 0, 0, 0, 0, 0;

  VectorXd sample_x(VectorXd::LinSpaced(9, 0, 4));
  VectorXd out_sample_f, out_sample_df, out_sample_d2f, out_sample_d3f;

  splines::hermite_cubic_spline_value(x, f, df, sample_x, out_sample_f, out_sample_df, out_sample_d2f, out_sample_d3f);

  cout << "x: " << x.transpose() << '\n';
  cout << "f: " << f.transpose() << '\n';
  cout << "df: " << df.transpose() << '\n';
  cout << "======================\n";
  cout << "sample_x: " << sample_x.transpose() << '\n';
  cout << "out_sample_f: " << out_sample_f.transpose() << '\n';
  cout << "out_sample_df: " << out_sample_df.transpose() << '\n';
  cout << "out_sample_d2f: " << out_sample_d2f.transpose() << '\n';
  cout << "out_sample_d3f: " << out_sample_d3f.transpose() << '\n';

  return 0;
}