#pragma once

#include <Eigen/Dense>

namespace splines {

using namespace Eigen;

// Eigen-ized version of original hermite_cubic_spline_value
void hermite_cubic_spline_value(
  const VectorXd &x, const VectorXd &f, const VectorXd &df,
  const VectorXd &sample_x, VectorXd &out_sample_f,
  VectorXd &out_sample_df, VectorXd &out_sample_d2f, VectorXd &out_sample_d3f
);

// Spline interpolation in many coordinates
// matrices are arranged as (num_points x dim), so their columns are interpolated separately
void multi_hermite_cubic_spline_value(
  const VectorXd &x, const MatrixXd &f, const MatrixXd &df,
  const VectorXd &sample_x, MatrixXd &out_sample_f,
  MatrixXd &out_sample_df, MatrixXd &out_sample_d2f, MatrixXd &out_sample_d3f
);


} // namespace splines
