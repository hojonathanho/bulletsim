#pragma once
#include <vector>
#include <Eigen/Dense>
#include <LinearMath/btTransform.h>
template <class T>
struct BasicArray {
public:
  std::vector<T> m_data;
  int m_nRow;
  int m_nCol;

  BasicArray() : m_nRow(0), m_nCol(0) {}
  BasicArray(int nRow, int nCol) : m_nRow(nRow), m_nCol(nCol) {m_data.resize(m_nRow*m_nCol);}
  BasicArray(const BasicArray& x) {
    m_nRow(x.m_nRow);
    m_nCol(x.m_nCol);
    m_data = x.m_data;
  }
  void resize(int nRow, int nCol) {
	  m_nRow = nRow;
	  m_nCol = nCol;
	  m_data.resize(m_nRow * m_nCol);
  }
  
  int rows() const {return m_nRow;}
  int cols() const {return m_nCol;}

  const T& at(int row, int col) const { return m_data.at(row*m_nCol + col); }
  T& at(int row, int col) { return m_data.at(row*m_nCol + col); }

  std::vector<T> col(int col) {
    std::vector<T> out;
    for (int row=0; row < m_nRow; row++) out.push_back(at(row,col));
    return out;
  }

  std::vector<T> row(int row) {
    std::vector<T> out;
    for (int col=0; col < m_nCol; col++) out.push_back(at(row,col));
    return out;
  }
  
};

std::vector<double> toDoubleVec(const Eigen::VectorXd& in);
inline Eigen::Vector3d toVector3d(const btVector3& in) {return Eigen::Vector3d(in.x(), in.y(), in.z());}
inline Eigen::Vector4d toVector4d(const btQuadWord& in) {return Eigen::Vector4d(in.x(), in.y(), in.z(), in.w());}
inline Eigen::VectorXd toVectorXd(const std::vector<double>& in) { return Eigen::Map<const Eigen::VectorXd>(in.data(), in.size(), 1); }
