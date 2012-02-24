#pragma once
#include <vector>
#include <iostream>
#include <Eigen/Dense>

struct IndVal {
  IndVal(int ind_, float val_) : ind(ind_), val(val_) {}
  IndVal() : ind(0), val(0) {}
  int ind;
  float val;
};

typedef std::vector<IndVal> SparseVector;
typedef std::vector<SparseVector> SparseArray;
float vecSum(const SparseVector& vec);
SparseVector mulVec(const SparseVector& in, float x);

SparseArray normalizeRows(const SparseArray&);
SparseArray toSparseArray(Eigen::MatrixXf, float cutoff);

std::ostream &operator<<(std::ostream &stream, SparseArray&);
std::ostream &operator<<(std::ostream &stream, SparseVector&);
std::ostream &operator<<(std::ostream &stream, IndVal&);
