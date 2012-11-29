#include "splines.h"

namespace splines {

void multi_hermite_cubic_spline_value(
  const ArrayXd &x, const MatrixXd &f, const MatrixXd &df,
  const ArrayXd &sample_x, MatrixXd &out_sample_f,
  MatrixXd &out_sample_df, MatrixXd &out_sample_d2f, MatrixXd &out_sample_d3f) {

  const int dim = f.cols();
  assert(df.cols() == dim);
  assert(x.size() == f.rows());
  assert(x.size() == df.rows());

  const int samples = sample_x.size();
  out_sample_f.resize(samples, dim);
  out_sample_df.resize(samples, dim);
  out_sample_d2f.resize(samples, dim);
  out_sample_d3f.resize(samples, dim);

  ArrayXd col_sample_f, col_sample_df, col_sample_d2f, col_sample_d3f;
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
