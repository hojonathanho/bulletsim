#pragma once
#include <Eigen/Dense>
#include <vector>

Eigen::VectorXi searchsorted(const Eigen::VectorXf& x, const Eigen::VectorXf& y);



template<typename T>
std::vector<T> interp(const Eigen::VectorXf& xNew, const Eigen::VectorXf& xOld, const std::vector<T>& yOld, T (*blend)(T,T,float)) {

  int nNew = xNew.size();
  int nOld = xOld.size();
  std::vector<T> yNew(nNew);
  Eigen::VectorXi new2old = searchsorted(xNew, xOld);
  for (int iNew=0; iNew < nNew; iNew++) {
    int iOldAbove = new2old(iNew);
    if (iOldAbove == 0) 
      yNew[iNew] = yOld[0];
    else if (iOldAbove == nOld)
      yNew[iNew] = yOld[nOld-1];
    else
      yNew[iNew] = blend(yOld[iOldAbove-1], yOld[iOldAbove], (xNew(iNew) - xOld(iOldAbove-1)) / (xOld(iOldAbove) - xOld(iOldAbove-1)));
    }
  return yNew;
}


Eigen::MatrixXf interp2d(const Eigen::VectorXf& xNew, const Eigen::VectorXf& xOld, const Eigen::MatrixXf& yOld);
