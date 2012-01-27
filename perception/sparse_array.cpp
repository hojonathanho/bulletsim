#include "sparse_array.h"
#include <boost/foreach.hpp>
using namespace std;

SparseArray toSparseArray(Eigen::MatrixXf in, float cutoff) {
  SparseArray out(in.rows());
  for (int row = 0; row < in.rows(); row++) {
    for (int col = 0; col < in.cols(); col++) {
      if (in(row,col) > cutoff) out[row].push_back(IndVal(col, in(row,col)));
    }
  }
  return out;
}

float vecSum(const SparseVector& vec) {
  float total=0;
  BOOST_FOREACH(const IndVal& iv, vec) total += iv.val;
  return total;
}

SparseVector mulVec(const SparseVector& in, float x) {
  SparseVector out(in.size());
  for (int i=0; i<in.size(); i++) {
    const IndVal& iv = in[i];
    out[i] = IndVal(iv.ind, iv.val*x);
  }
}

SparseArray normalizeRows(const SparseArray& in) {
  SparseArray out(in.size());
  for (int iRow=0; iRow < in.size(); iRow++) {
    SparseVector inRow = in[iRow];
    SparseVector outRow = out[iRow];
    outRow.reserve(inRow.size());
    float rowSum = vecSum(inRow);
    BOOST_FOREACH(IndVal& iv, inRow) outRow.push_back(IndVal(iv.ind,iv.val/rowSum));
  }
  return out;
}


ostream &operator<<(ostream &stream, IndVal& iv) {
  stream << "(" << iv.ind << ", " << iv.val << ")";
  return stream;
}

ostream &operator<<(ostream &stream, SparseVector& vec) {
  BOOST_FOREACH(IndVal& iv, vec) stream << iv << " ";
  return stream;
}


ostream &operator<<(ostream &stream, SparseArray& arr) {
  BOOST_FOREACH(SparseVector& vec, arr) stream << vec << endl;
  return stream;
}

