#pragma once
#include <Eigen/Dense>

class TrajPlotter {
public:
  virtual void plotTraj(const Eigen::MatrixXd& traj) = 0;
  virtual void clear() {
  }
  virtual ~TrajPlotter() {
  }
};