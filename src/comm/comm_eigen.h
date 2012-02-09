#include "comm2.h"
#include <Eigen/Dense>

struct EigenMessage : Message {
  Eigen::MatrixXf m_data;
  EigenMessage() : Message() {}
  EigenMessage(Eigen::MatrixXf& mat) : Message(), m_data(mat) {}
  EigenMessage(Eigen::MatrixXf& mat, Value info) : Message(info), m_data(mat) {}
  void writeDataTo(path) const;
  void readDataFrom(path);
};
