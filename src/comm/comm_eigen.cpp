#include "comm_eigen.h"
#include <iostream>
using namespace std;

void EigenMessage::writeDataTo(fs::path p) const {
  ofstream outfile(p.string().c_str());
  ENSURE(!outfile.fail());
  const float* floatData = m_data.data();
  const char* charData = reinterpret_cast<const char*>(floatData);

  int nRow = m_data.rows();
  int nCol = m_data.cols();
  int nChar = m_data.rows() * m_data.cols() * sizeof(float);

  outfile.write(reinterpret_cast<char*>(&nRow), sizeof(nRow));
  outfile.write(reinterpret_cast<char*>(&nCol), sizeof(nCol));
  outfile.write(charData, nChar);
  outfile.close();
}

void EigenMessage::readDataFrom(fs::path p) {
  ifstream infile(p.string().c_str());
  ENSURE(!infile.fail());

  int nRow;
  int nCol;
  infile.read(reinterpret_cast<char*>(&nRow), sizeof(nRow));
  infile.read(reinterpret_cast<char*>(&nCol), sizeof(nCol));

  m_data.resize(nRow, nCol);
  int nChar = nRow*nCol*sizeof(float);
  infile.read(reinterpret_cast<char*>(m_data.data()), nChar);
  infile.close();
}
