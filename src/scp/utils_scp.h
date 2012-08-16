#pragma once
#include <vector>
#include <Eigen/Dense>

template <class T>
struct BasicArray {
public:
  std::vector<T> m_data;
  int m_nRow;
  int m_nCol;

  BasicArray(int nRow, int nCol) : m_nRow(nRow), m_nCol(nCol) {m_data.resize(m_nRow*m_nCol);}

  T& at(int row, int col) { return m_data.at(row*m_nCol + col); }

  std::vector<T> getCol(int col) {
    std::vector<T> out;
    for (int row=0; row < m_nRow; row++) out.push_back(at(row,col));
    return out;
  }

  std::vector<T> getRow(int row) {
    std::vector<T> out;
    for (int col=0; col < m_nCol; col++) out.push_back(at(row,col));
    return out;
  }

};


std::vector<double> toDoubleVec(const Eigen::VectorXf& in);
