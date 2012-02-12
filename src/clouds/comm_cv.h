#include "comm/comm.h"
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

struct ImageMessage : Message {
  cv::Mat m_data;
  ImageMessage() : Message() {}
  ImageMessage(cv::Mat& mat) : Message(), m_data(mat) {}
  ImageMessage(cv::Mat& mat, Value info) : Message(info), m_data(mat) {}
  void writeDataTo(path) const;
  void readDataFrom(path);
};
