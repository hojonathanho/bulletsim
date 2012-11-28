#pragma once
#include <vector>
#include <Eigen/Dense>
#include <LinearMath/btTransform.h>
#include <btBulletCollisionCommon.h>
#include "gurobi_c++.h"
#include <string>

using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector4d;


template<class T>
struct BasicArray {

  std::vector<T> m_data;
  int m_nRow;
  int m_nCol;

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
  BasicArray block(int startRow, int startCol, int nRow, int nCol) const {
    BasicArray out;
    out.resize(nRow, nCol);
    for (int iRow = 0; iRow < nRow; ++iRow) {
      for (int iCol = 0; iCol < nCol; ++iCol) {
        out(iRow, iCol) = at(iRow+startRow, iCol + startCol);
      }
    }
    return out;
  }
  vector<T> rblock(int startRow, int startCol, int nCol) const {
    vector<T> out(nCol);
		for (int iCol = 0; iCol < nCol; ++iCol) {
			out[iCol] = at(startRow, iCol + startCol);
    }
    return out;
  }
  BasicArray middleRows(int start, int n) {
    BasicArray out;
    out.resize(n, m_nCol);
    for (int i=start; i < start+n; ++i) {
      for (int j=0; j < m_nCol; ++j) {
        out(i,j) = at(i,j);
      }
    }
    return out;
  }
  BasicArray topRows(int n) {
    return middleRows(0,n);
  }
  BasicArray bottomRows(int n) {
    return middleRows(m_nRow-n, n);
  }

  const T& at(int row, int col) const {
    return m_data.at(row * m_nCol + col);
  }
  T& at(int row, int col) {
    return m_data.at(row * m_nCol + col);
  }
  const T& operator()(int row, int col) const {
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
typedef BasicArray<GRBLinExpr> ExprMatrix;
typedef vector<GRBLinExpr> ExprVector;
typedef vector<GRBQuadExpr> QExprVector;
ExprVector operator+(const VarVector&, const VarVector&);

void setValsToVars(VarVector& vars, VectorXd& vals);
void setValsToVars(VarArray& vars, MatrixXd& vals);
void setValsToVars(VarVector& vars, Vector3d& vals);

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
inline float randf() {return (float)rand()/(float)RAND_MAX;}

GRBLinExpr makeDerivExpr(const VectorXd& grad, const VarVector& vars, const VectorXd& curvals);

VectorXd arange(int n);
MatrixXd linearInterp(const VectorXd& start, const VectorXd& end, int nSteps);
VectorXd concatenate(VectorXd x0, VectorXd x1);

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


#define DEBUG_PRINT(exp) std::cout << #exp << ": " << exp << std::endl;

