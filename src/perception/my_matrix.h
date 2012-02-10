#pragma once
#include "utils/my_exceptions.h"

template <class T> 
class MyMatrix {
public:
  vector<T> m_data;
  int m_nRow;
  int m_nCol;

  MyMatrix(int nRow, int nCol) : m_nRow(nRow), m_nCol(nCol) {m_data.resize(m_nRow*m_nCol);}

  T& at(int row, int col) { return m_data.at(row*m_nCol + col); }

  vector<T> getCol(int col) {
    vector<T> out;
    for (int row=0; row < m_nRow; row++) out.push_back(at(row,col));
    return out;
  }

  vector<T> getRow(int row) {
    vector<T> out;
    for (int col=0; col < m_nCol; col++) out.push_back(at(row,col));
    return out;
  }

};

