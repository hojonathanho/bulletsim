#include <Eigen/Dense>
#include <utility>
#include <algorithm>
#include <boost/foreach.hpp>
#include <iostream>
#include <vector>
using namespace Eigen;
using namespace std;




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

template<typename T>
vector<T> interp(const VectorXf& xNew, const VectorXf& xOld, const vector<T>& yOld, T (*blend)(T,T,float)) {

  int nNew = xNew.size();
  int nOld = xOld.size();
  vector<T> yNew(nNew);
  VectorXi new2old = searchsorted(xNew, xOld);
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

MatrixXf interp2f(const VectorXf& xNew, const VectorXf& xOld, const MatrixXf& yOld) {

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

MatrixXd interp2d(const VectorXd& xNew, const VectorXd& xOld, const MatrixXd& yOld) {

  int nNew = xNew.size();
  int nOld = xOld.size();
  MatrixXd yNew(nNew, yOld.cols());
  VectorXi new2old = searchsorted(xNew, xOld);
  for (int iNew=0; iNew < nNew; iNew++) {
    int iOldAbove = new2old(iNew);
    if (iOldAbove == 0) 
      yNew.row(iNew) = yOld.row(0);
    else if (iOldAbove == nOld)
      yNew.row(iNew) = yOld.row(nOld-1);
    else {
      double t = (xNew(iNew) - xOld(iOldAbove-1)) / (xOld(iOldAbove) - xOld(iOldAbove-1));
      yNew.row(iNew) = yOld.row(iOldAbove-1)*(1-t) + yOld.row(iOldAbove)*t;
    }
  }
  return yNew;
}



float blendFloats(float a, float b, float t) {
  return a*(1-t) + b*t;

}




int main() {
  VectorXf x(6); x << -5, -4,.5, 3, 3.1, 3.2;
  VectorXf y(4); y << 0,  1,   2,  3;

  VectorXi inds = searchsorted(x,y);

  vector<float> yOld;
  yOld.push_back(0);
  yOld.push_back(1);
  VectorXf xOld(2); xOld << 0,1;
  VectorXf xNew(5); xNew << -1,0,.25,1,2;

  vector<float> yNew = interp(xNew, xOld, yOld, &blendFloats);

  for (int i=0; i < yNew.size(); i++) cout << yNew[i] << " ";
  cout << endl;

  MatrixXf m(2,1); m << 0,1;
  MatrixXf mnew = interp2f(xNew, xOld, m);
  cout << endl << mnew << endl;

}
