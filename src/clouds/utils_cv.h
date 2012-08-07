#pragma once
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>

cv::Mat toSingleChannel(const cv::Mat&);

std::string typeAsString(int type);

// input and output images are binaries (i.e 0's and 255's). src gets modified.
cv::Mat connectedComponentsFilter(cv::Mat src, int min_pix, int dist_pix);

//src and dst are binary images. removes sparse pixels and small connected components.
cv::Mat sparseSmallFilter(cv::Mat src, int erode, int dilate, int min_connected_components, int tol_connected_components);

namespace cv {
	void thresholdMulti(const Mat& src, Mat& dst, Scalar thresh, double maxVal, int thresholdType);
}

//depth and depth_bg are CV_32FC1
//the returned image is of type CV_8UC1
//the threshold is a relative quantity
cv::Mat backgroundSubtractorDepthMask(cv::Mat depth, std::vector<cv::Mat> depth_bg, double depth_th=0.004, int erode=1, int dilate=2, int min_connected_components=300, int tol_connected_components=2);

//rgb and rgb_bg are CV_8UC3
//the returned image is of type CV_8UC1
//the threshold is an absolute quantity
//Example to get foreground and background
//cv::Mat foreground_mask = backgroundSubtractorColorMask(rgb, rgb_bg);
//cv::Mat foreground, background;
//cv::merge(vector<cv::Mat>(3, foreground_mask), foreground_mask);
//cv::multiply(rgb, foreground_mask, foreground, 1/255.0);
//cv::subtract(rgb, foreground, background);
cv::Mat backgroundSubtractorColorMask(cv::Mat color, std::vector<cv::Mat> color_bg, cv::Scalar trans_th = cv::Scalar(255,15,15), int space_transformation=CV_BGR2Lab, int erode=1, int dilate=2, int min_connected_components=300, int tol_connected_components=2);

// input is an bgr image and output is a binary mask (i.e. 0's are likely to not be skin and 255's are likely to be skin). src doesn't get modified.
cv::Mat skinMask(cv::Mat src);
