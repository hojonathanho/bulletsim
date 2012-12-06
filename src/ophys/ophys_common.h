#ifndef __OPTPHYSICS_COMMON_H__
#define __OPTPHYSICS_COMMON_H__

#include <vector>
#include <boost/multi_array.hpp>
#include <gurobi_c++.h>
#include <Eigen/StdVector>
#include <boost/random.hpp>
#include <ctime>
#include <btBulletDynamicsCommon.h>

namespace ophys {

using namespace Eigen;

typedef std::vector<GRBVar *> VarPVec;
typedef std::vector<GRBVar> VarVec;
typedef std::vector<GRBTempConstr> ConstrVec;
typedef boost::multi_array<GRBVar, 2> VarMatrix;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, Eigen::Dynamic, 7> MatrixX7d;

inline GRBQuadExpr square(const GRBLinExpr &e) { return e*e; }

template<typename T>
inline T square(const T& e) { return e*e; }

inline vector<double> toStlVec(const VectorXd &v) {
  vector<double> out(v.size());
  for (int i = 0; i < v.size(); ++i) {
    out[i] = v[i];
  }
  return out;
}

inline VectorXd toEigVec(const vector<double> &v) {
  VectorXd out(v.size());
  for (int i = 0; i < v.size(); ++i) {
    out[i] = v[i];
  }
  return out;
}

inline Vector3d toEigVec(const btVector3 &v) {
  return Vector3d(v.x(), v.y(), v.z());
}

template<typename EigenType>
struct EigVector {
  typedef std::vector<EigenType, Eigen::aligned_allocator<EigenType> > type;
};

template<typename VectorType>
static void addNoise(VectorType &x, double mean, double stdev) {
  static boost::mt19937 rng(static_cast<unsigned> (std::time(0)));
  boost::normal_distribution<double> norm_dist(mean, stdev);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > gen(rng, norm_dist);
  for (int i = 0; i < x.size(); ++i) {
    x[i] += gen();
  }
}

inline Vector3d centroid(const MatrixX3d &points) {
  return points.colwise().sum() / (double)points.rows();
}


// left indices: [ 15, 16, 17, 18, 19, 20, 21 ] vals: [ 0.870844, 0.163559, 0.6, -1.28408, 2.25124, -0.827074, 3.09148 ]
// right indices: [ 27, 28, 29, 30, 31, 32, 33 ] vals: [ -0.793454, 0.221086, -0.6, -1.54165, -2.4981, -0.989481, -2.842 ]
inline vector<double> pr2LeftNeutralPos() {
  static const double vals[] = { .870844, 0.163559, 0.6, -1.28408, 2.25124, -0.827074, 3.09148 };
  vector<double> v(7);
  for (int i = 0; i < 7; ++i) {
    v[i] = vals[i];
  }
  return v;
}
inline vector<double> pr2RightNeutralPos() {
  //static const double vals[] = { -0.793454, 0.221086, -0.6, -1.54165, -2.4981, -0.989481, -2.842 };
  static const double vals[] = { 0.151873, 1.06237, 0, -1.39138, 2.92417, -0.770895, -2.917 };
  vector<double> v(7);
  for (int i = 0; i < 7; ++i) {
    v[i] = vals[i];
  }
  return v;
}


} // namespace ophys

#endif // __OPTPHYSICS_COMMON_H__
