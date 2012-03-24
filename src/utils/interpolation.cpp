#include <btBulletDynamicsCommon.h>
#include "interpolation.h"

using namespace std;
using namespace Eigen;

VectorXi searchsorted(const VectorXf& x, const VectorXf& y) {
  // y(i-1) <= x(out(i)) < y(i)
  int nX = x.size();
  int nY = y.size();

  VectorXi out(nX);
  int iY=0;
  for (int iX=0; iX < nX; iX++) {
    while (iY < nY && x(iX) > y(iY)) iY++;
    out(iX) = iY;
  }
  return out;
}

MatrixXf interp2d(const VectorXf& xNew, const VectorXf& xOld, const MatrixXf& yOld) {

  int nNew = xNew.size();
  int nOld = xOld.size();
  MatrixXf yNew(nNew, yOld.cols());
  VectorXi new2old = searchsorted(xNew, xOld);
  for (int iNew=0; iNew < nNew; iNew++) {
    int iOldAbove = new2old(iNew);
    if (iOldAbove == 0) 
      yNew.row(iNew) = yOld.row(0);
    else if (iOldAbove == nOld)
      yNew.row(iNew) = yOld.row(nOld-1);
    else {
      float t = (xNew(iNew) - xOld(iOldAbove-1)) / (xOld(iOldAbove) - xOld(iOldAbove-1));
      yNew.row(iNew) = yOld.row(iOldAbove-1)*(1-t) + yOld.row(iOldAbove)*t;
    }
  }
  return yNew;
}
