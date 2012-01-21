#include "sparse_array.h"
#include <boost/foreach.hpp>

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


