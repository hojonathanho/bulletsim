#include "splines.h"

#include "hermite_cubic.hpp"

namespace splines {

void hermite_cubic_spline_value(
  const VectorXd &x, const VectorXd &f, const VectorXd &df,
  const VectorXd &sample_x, VectorXd &out_sample_f,
  VectorXd &out_sample_df, VectorXd &out_sample_d2f, VectorXd &out_sample_d3f
  ) {

  int n = x.size();
  assert(f.size() == n);
  assert(df.size() == n);

  int m = sample_x.size();
  out_sample_f.resize(m);
  out_sample_df.resize(m);
  out_sample_d2f.resize(m);
  out_sample_d3f.resize(m);

  // I hope this is safe
  ::hermite_cubic_spline_value(
    n, (double*)x.data(), (double*)f.data(), (double*)df.data(),
    m, (double*)sample_x.data(), (double*)out_sample_f.data(), (double*)out_sample_df.data(), (double*)out_sample_d2f.data(), (double*)out_sample_d3f.data()
  );
}

void multi_hermite_cubic_spline_value(
  const VectorXd &x, const MatrixXd &f, const MatrixXd &df,
  const VectorXd &sample_x, MatrixXd &out_sample_f,
  MatrixXd &out_sample_df, MatrixXd &out_sample_d2f, MatrixXd &out_sample_d3f) {

  int dim = f.cols();
  assert(df.cols() == dim);
  assert(x.size() == f.rows());
  assert(x.size() == df.rows());

  int samples = sample_x.size();
  out_sample_f.resize(samples, dim);
  out_sample_df.resize(samples, dim);
  out_sample_d2f.resize(samples, dim);
  out_sample_d3f.resize(samples, dim);

  VectorXd col_sample_f, col_sample_df, col_sample_d2f, col_sample_d3f;
  for (int i = 0; i < dim; ++i) {
    hermite_cubic_spline_value(
      x, f.col(i).transpose(), df.col(i).transpose(),
      sample_x, col_sample_f, col_sample_df, col_sample_d2f, col_sample_d3f
    );
    out_sample_f.col(i) = col_sample_f.transpose();
    out_sample_df.col(i) = col_sample_df.transpose();
    out_sample_d2f.col(i) = col_sample_d2f.transpose();
    out_sample_d3f.col(i) = col_sample_d3f.transpose();
  }
}

} // namespace splines
