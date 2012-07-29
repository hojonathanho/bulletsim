#pragma once
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>

cv::Mat toSingleChannel(const cv::Mat&);

// input and output images are binaries (i.e 0's and 255's). src gets modified.
cv::Mat connectedComponentsFilter(cv::Mat src, int min_pix);

// input is an bgr image and output is a binary mask (i.e. 0's are likely to not be skin and 255's are likely to be skin). src doesn't get modified.
cv::Mat skinMask(cv::Mat src);
