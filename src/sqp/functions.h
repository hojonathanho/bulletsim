#pragma once
#include <Eigen/Dense>
#include <stdexcept>

class ScalarValuedFunction {
public:
  virtual float calcVal(const Eigen::VectorXd& in)=0;
  virtual Eigen::VectorXd calcDeriv(const Eigen::VectorXd& in, Eigen::VectorXd& deriv) { throw std::runtime_error("not implemented"); }
  virtual void calcValAndDeriv(const Eigen::VectorXd& in, float& val, Eigen::MatrixXd& deriv) { throw std::runtime_error("not implemented"); }
  virtual Eigen::VectorXd calcNumDeriv(const Eigen::VectorXd& in, float spacing);
};
