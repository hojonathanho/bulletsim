#pragma once

#include <Eigen/Dense>
#include "hermite_cubic.hpp"

namespace splines {

using namespace Eigen;

// Eigen-ized version of Burkardt's hermite_cubic_spline_value
template<typename Derived1, typename Derived2>
inline void hermite_cubic_spline_value(
  const ArrayXd &x, const DenseBase<Derived1> &f, const DenseBase<Derived2> &df,
  const ArrayXd &sample_x, ArrayXd &out_sample_f,
  ArrayXd &out_sample_df, ArrayXd &out_sample_d2f, ArrayXd &out_sample_d3f
   ) {

  const int n = x.size();
  assert(f.size() == n);
  assert(df.size() == n);

  const int m = sample_x.size();

  ArrayXd x1(m), x2(m), f1(m), f2(m), d1(m), d2(m);
  for (int i = 0; i < m; ++i) {
    int left = m/2;
    r8vec_bracket3(n, (double*)x.data(), sample_x[i], &left); // TODO: re-implement this
    x1[i] = x[left]; x2[i] = x[left+1];
    f1[i] = f[left]; f2[i] = f[left+1];
    d1[i] = df[left]; d2[i] = df[left+1];
  }
  ArrayXd h = x2 - x1;
  ArrayXd fp = (f2 - f1)/h;
  ArrayXd c2 = -(2.0*d1 - 3.0*fp + d2)/h;
  ArrayXd c3 = (d1 - 2.0*fp + d2)/h/h;
  ArrayXd dsx = sample_x - x1;

  out_sample_f = f1 + dsx*(d1 + dsx*(c2 + dsx*c3));
  out_sample_df = d1 + dsx*(2.0*c2 + dsx*3.0*c3);
  out_sample_d2f = 2.0*c2 + dsx*6.0*c3;
  out_sample_d3f = 6.0*c3;
}


// Spline interpolation in many coordinates
// matrices are arranged as (num_points x dim), so their columns are interpolated separately
// void multi_hermite_cubic_spline_value(
//   const VectorXd &x, const MatrixXd &f, const MatrixXd &df,
//   const VectorXd &sample_x, MatrixXd &out_sample_f,
//   MatrixXd &out_sample_df, MatrixXd &out_sample_d2f, MatrixXd &out_sample_d3f
// );
void multi_hermite_cubic_spline_value(
  const ArrayXd &x, const MatrixXd &f, const MatrixXd &df,
  const ArrayXd &sample_x, MatrixXd &out_sample_f,
  MatrixXd &out_sample_df, MatrixXd &out_sample_d2f, MatrixXd &out_sample_d3f
);

} // namespace splines
