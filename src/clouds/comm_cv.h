#pragma once
#include "comm/comm.h"
using namespace comm;
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

struct ImageMessage : Message {
  cv::Mat m_data;
  ImageMessage() : Message() {}
  ImageMessage(cv::Mat& mat) : Message(), m_data(mat) {}
  ImageMessage(cv::Mat& mat, Json::Value info) : Message(info), m_data(mat) {}
  void writeDataTo(fs::path) const;
  void readDataFrom(fs::path);
};
