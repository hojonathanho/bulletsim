#include "utils_cv.h"
#include <vector>
#include <iostream>
using namespace std;

cv::Mat toSingleChannel(const cv::Mat& in) {
  vector<cv::Mat> channels;
  cv::split(in, channels);
  return channels[0];
}
