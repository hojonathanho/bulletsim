#include "comm_cv.h"
#include <opencv2/highgui/highgui.hpp>
#include "utils/my_exceptions.h"

void ImageMessage::writeDataTo(path p) const {
  bool success = cv::imwrite(p.string(), m_data);
  if (!success) throw FileOpenError(p.string()); // todo: error might be for some other reason
}

void ImageMessage::readDataFrom(path p) {
  m_data = cv::imread(p.string(),0);
  if (m_data.data == NULL) throw FileOpenError(p.string());
}
