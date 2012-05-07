#include "comm.h"
using namespace comm;
#include <Eigen/Dense>

struct EigenMessage : Message {
  Eigen::MatrixXf m_data;
  EigenMessage() : Message() {}
  EigenMessage(Eigen::MatrixXf& mat) : Message(), m_data(mat) {}
  EigenMessage(Eigen::MatrixXf& mat, Json::Value info) : Message(info), m_data(mat) {}
  void writeDataTo(fs::path) const;
  void readDataFrom(fs::path);
};
