#pragma once
#include <vector>
#include <Eigen/Dense>
#include <LinearMath/btTransform.h>
#include <btBulletCollisionCommon.h>
#include "gurobi_c++.h"
#include <string>

template<class T>
struct BasicArray {

private:
  std::vector<T> m_data;
  int m_nRow;
  int m_nCol;

public:

  BasicArray() :
    m_nRow(0), m_nCol(0) {
  }
  BasicArray(int nRow, int nCol) :
    m_nRow(nRow), m_nCol(nCol) {
    m_data.resize(m_nRow * m_nCol);
  }
  BasicArray(const BasicArray& x) :
    m_nRow(x.m_nRow), m_nCol(x.m_nCol), m_data(x.m_data) {
  }
  void resize(int nRow, int nCol) {
    m_nRow = nRow;
    m_nCol = nCol;
    m_data.resize(m_nRow * m_nCol);
  }

  int rows() const {
    return m_nRow;
  }
  int cols() const {
    return m_nCol;
  }

  const T& at(int row, int col) const {
    return m_data.at(row * m_nCol + col);
  }
  T& at(int row, int col) {
    return m_data.at(row * m_nCol + col);
  }
  T& operator()(int row, int col) {
    return m_data.at(row * m_nCol + col);
  }

  std::vector<T> col(int col) {
    std::vector<T> out;
    out.reserve(m_nRow);
    for (int row = 0; row < m_nRow; row++)
      out.push_back(at(row, col));
    return out;
  }

  std::vector<T> row(int row) {
    std::vector<T> out;
    out.reserve(m_nCol);
    for (int col = 0; col < m_nCol; col++)
      out.push_back(at(row, col));
    return out;
  }

  std::vector<T> flatten() {
    return std::vector<T>(m_data.begin(), m_data.end());
  }

  T* data() {
    return m_data.data();
  }
  T* data() const {
    return m_data.data();
  }

};

typedef BasicArray<GRBVar> VarArray;
typedef std::vector<GRBVar> VarVector;
typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;

std::vector<double> toDoubleVec(const Eigen::VectorXd& in);
inline Eigen::Vector3d toVector3d(const btVector3& in) {
  return Eigen::Vector3d(in.x(), in.y(), in.z());
}
inline Eigen::Vector4d toVector4d(const btQuadWord& in) {
  return Eigen::Vector4d(in.x(), in.y(), in.z(), in.w());
}
inline Eigen::VectorXd toVectorXd(const std::vector<double>& in) {
  return Eigen::Map<const Eigen::VectorXd>(in.data(), in.size(), 1);
}

void printAllConstraints(const GRBModel& model);

inline double pospart(double x) {
  return (x > 0) ? x : 0;
}
inline double sq(double x) {
  return x * x;
}
inline double clip(double x, double lo, double hi) {
  return x < lo ? lo :
                x > hi ? hi :
                       x;
}

inline std::string base_filename(char* in) {
  std::string s(in);
  size_t ind = s.rfind('/');
  if (ind == std::string::npos)
    return s;
  else
    return s.substr(ind + 1, std::string::npos);
}

#define ASSERT_ALMOST_EQUAL(a,b,tol)\
do {\
  if ( !( 2 * fabs(a-b) / (fabs(a)+fabs(b)) < tol) )  {\
    printf("%s != %s (%.4e != %.4e) in file %s at line %i\n", #a, #b, a, b, base_filename(__FILE__).c_str(), __LINE__ );\
    exit(1);\
  }\
} while(0)

#define ASSERT_ALMOST_EQUAL2(a,b,reltol, abstol)\
do  {\
    if ( !( 2 * fabs(a-b) / (fabs(a)+fabs(b)) < reltol || fabs(a-b) < abstol) )  {\
      printf("%s != %s (%.4e != %.4e) in file %s at line %i\n", #a, #b, a, b, base_filename(__FILE__).c_str(), __LINE__ );\
      exit(1);\
        }\
  } while(0)


struct Collision {
  const btCollisionObject* m_obj0;
  const btCollisionObject* m_obj1;
  btVector3 m_world0;
  btVector3 m_world1;
  btVector3 m_normal;
  btScalar m_distance;
  Collision(const btCollisionObject* obj0, const btCollisionObject* obj1, const btVector3& world0, const btVector3& world1, const btVector3& normal, btScalar distance) :
    m_obj0(obj0), m_obj1(obj1), m_world0(world0), m_world1(world1), m_normal(normal), m_distance(distance) {
  }
  Collision(const Collision& c) :
    m_obj0(c.m_obj0), m_obj1(c.m_obj1), m_world0(c.m_world0), m_world1(c.m_world1), m_normal(c.m_normal), m_distance(c.m_distance) {
  }
};

struct LinkCollision {
  double dist;
  int linkInd;
  btVector3 point; // world position
  btVector3 normal; // world normal
  float frac; // only used in continuous collision detection
  LinkCollision(double dist_, int linkInd_, const btVector3& point_, const btVector3& normal_):
    dist(dist_), linkInd(linkInd_), point(point_), normal(normal_) {}
};

typedef std::vector<LinkCollision> CartCollInfo;
typedef std::vector<CartCollInfo> TrajCartCollInfo;
struct JointCollInfo {
  std::vector<Eigen::VectorXd> jacs;
  std::vector<double> dists;
};
typedef std::vector< JointCollInfo > TrajJointCollInfo;

#define DEBUG_PRINT(exp) std::cout << #exp << ": " << exp << std::endl;

