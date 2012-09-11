#include "utils_cv.h"
#include <vector>
#include <iostream>
#include "utils/utils_vector.h"
#include <stdexcept>
using namespace std;

cv::Mat toSingleChannel(const cv::Mat& in) {
  vector<cv::Mat> channels;
  cv::split(in, channels);
  return channels[0];
}

string typeAsString(int type) {
	int n_depths = 7;
	string depths_a[] = {"CV_8U", "CV_8S", "CV_16U", "CV_16S", "CV_32S", "CV_32F", "CV_64F"};
	if (CV_MAT_DEPTH(type) < n_depths) {
		return depths_a[CV_MAT_DEPTH(type)] + "C" + itoa(CV_MAT_CN(type));
	}
	return itoa(type);
}

// input and output images are binaries (i.e 0's and 255's). src gets modified.
cv::Mat connectedComponentsFilter(cv::Mat src, int min_pix, int dist_pix) {
	cv::Mat src_tmp;
	cv::dilate(src, src_tmp, cv::Mat(), cv::Point(-1, -1), dist_pix);
	cv::Mat dst = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);

	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;

	cv::findContours( src_tmp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

	// iterate through all the top-level contours,
	int idx = 0;
	for( ; idx >= 0; idx = hierarchy[idx][0] )
	{
		if (contours.size() == 0) break;
		double area = cv::contourArea(cv::Mat(contours[idx]));
		cv::Scalar color;
		if (area > min_pix) color = cv::Scalar(255);
		else color = cv::Scalar(0);
		cv::drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy );
	}
	cv::multiply(src, dst, dst, 1/255.0);
	return dst;
}

// input and output images are BGR images.
cv::Mat connectedComponentsFilterColor(cv::Mat src, int min_pix, int dist_pix) {
	cv::Mat src_gray;
	cv::cvtColor(src, src_gray, CV_BGR2GRAY);
	src_gray = src_gray > 0;
	src_gray = connectedComponentsFilter(src_gray, min_pix, dist_pix);
	cv::merge(vector<cv::Mat>(3, src_gray), src_gray);
	cv::multiply(src_gray, src, src, 1/255.0);
	return src;
}

//src and dst are binary images. removes sparse pixels and small connected components.
cv::Mat sparseSmallFilter(cv::Mat src, int erode, int dilate, int min_connected_components, int tol_connected_components) {
	cv::Mat dst = src.clone();
	cv::erode(dst, dst, cv::Mat(), cv::Point(-1, -1), erode);
	cv::dilate(dst, dst, cv::Mat(), cv::Point(-1, -1), dilate);
	dst = connectedComponentsFilter(dst, min_connected_components, tol_connected_components);
	cv::multiply(src, dst, dst, 1/255.0);
	return dst;
}

// input and output are BGR images.
cv::Mat sparseSmallFilterColor(cv::Mat src, int erode, int dilate, int min_connected_components, int tol_connected_components) {
	cv::Mat src_gray;
	cv::cvtColor(src, src_gray, CV_BGR2GRAY);
	src_gray = src_gray > 0;
	cv::erode(src_gray, src_gray, cv::Mat(), cv::Point(-1, -1), erode);
	cv::dilate(src_gray, src_gray, cv::Mat(), cv::Point(-1, -1), dilate);
	src_gray = connectedComponentsFilter(src_gray, min_connected_components, tol_connected_components);
	cv::merge(vector<cv::Mat>(3, src_gray), src_gray);
	cv::Mat dst;
	cv::multiply(src, src_gray, dst, 1/255.0);
	return dst;
}

cv::Mat toBinaryMask(cv::Mat image) {
	cv::Mat mask = image > 0;
	vector<cv::Mat> mask_channels;
	cv::split(mask, mask_channels);
	mask = mask_channels[0] & mask_channels[1] & mask_channels[2];
	return mask;
}

cv::Mat colorSpaceMask(cv::Mat cvmat, uint8_t minx, uint8_t maxx, uint8_t miny, uint8_t maxy, uint8_t minz, uint8_t maxz, int code) {
	cv::cvtColor(cvmat, cvmat, code);
  vector<cv::Mat> channels;
  cv::split(cvmat, channels);

  cv::Mat& x = channels[0];
  cv::Mat& y = channels[1];
  cv::Mat& z = channels[2];

  cv::Mat mask = (x > minx) & (x < maxx);
	if (miny > 0) mask &= (y >= miny);
	if (maxy < 255) mask &= (y <= maxy);
	if (minz > 0) mask &= (z >= minz);
	if (maxz < 255) mask &= (z <= maxz);

	return mask;
}

namespace cv {
	void thresholdMulti(const Mat& src, Mat& dst, Scalar thresh, double maxVal, int thresholdType) {
		vector<Mat> channels;
		split(src, channels);
		vector<Mat> thresholds(channels.size());
		dst = Mat::zeros(src.rows, src.cols, CV_8UC1);
		for (int i=0; i<thresholds.size(); i++) {
			threshold(channels[i], thresholds[i], thresh[i], maxVal, thresholdType);
			max(thresholds[i], dst, dst);
		}
	}
}

//depth and depth_bg are CV_32FC1
//the returned image is of type CV_8UC1
//the threshold is a relative quantity
cv::Mat backgroundSubtractorDepthMask(cv::Mat depth, vector<cv::Mat> depth_bg, double depth_th, int erode, int dilate, int min_connected_components, int tol_connected_components) {
	cv::Mat depth_bg_min = min(depth_bg).clone();
	cv::Mat depth_bg_max = max(depth_bg).clone();

	cv::Mat depth_diff = (depth < (depth_bg_min - depth_bg_min*depth_th)) | (depth > (depth_bg_max + depth_bg_max*depth_th));
	//cv::imshow("depth_diff_raw", depth_diff);
	depth_diff = sparseSmallFilter(depth_diff, erode, dilate, min_connected_components, tol_connected_components);
	//cv::imshow("depth_diff", depth_diff);

	return depth_diff;
}

//rgb and rgb_bg are CV_8UC3
//the returned image is of type CV_8UC1
//the threshold is an absolute quantity
//Example to get foreground and background
//cv::Mat foreground_mask = backgroundSubtractorColorMask(rgb, rgb_bg);
//cv::Mat foreground, background;
//cv::merge(vector<cv::Mat>(3, foreground_mask), foreground_mask);
//cv::multiply(rgb, foreground_mask, foreground, 1/255.0);
//cv::subtract(rgb, foreground, background);
cv::Mat backgroundSubtractorColorMask(cv::Mat color, vector<cv::Mat> color_bg, cv::Scalar trans_th, int space_transformation, int erode, int dilate, int min_connected_components, int tol_connected_components) {
	vector<cv::Mat> trans_bg(color_bg.size());
	for (int i=0; i<color_bg.size(); i++)
		cv::cvtColor(color_bg[i], trans_bg[i], space_transformation);

	cv::Mat trans_bg_min = min(trans_bg).clone();
	cv::Mat trans_bg_max = max(trans_bg).clone();

	cv::Mat trans;
	cv::cvtColor(color, trans, space_transformation);
	cv::Mat trans_diff = (trans < (trans_bg_min - trans_th)) | (trans > (trans_bg_max + trans_th));
	//cv::imshow("trans_diff_raw", trans_diff);
	vector<cv::Mat> trans_diff_channels;
	cv::split(trans_diff, trans_diff_channels);
	trans_diff = max(trans_diff_channels);
	trans_diff = sparseSmallFilter(trans_diff, erode, dilate, min_connected_components, tol_connected_components);
	//cv::imshow("trans_diff", trans_diff);

	return trans_diff;
}

// input is an bgr image and output is a binary mask (i.e. 0's are likely to not be skin and 255's are likely to be skin). src doesn't get modified.
// YCrCb thresholds from http://waset.org/journals/waset/v43/v43-91.pdf except for Ymin = 40
// merging from here http://www.csee.wvu.edu/~richas/papers/tkjse.pdf (?)
cv::Mat skinMask(cv::Mat src) {
	cv::Mat srcYCrCb(src.rows, src.cols, CV_8UC3);
	cv::cvtColor(src, srcYCrCb, CV_BGR2YCrCb);
	cv::Mat srcLab(src.rows, src.cols, CV_8UC3);
	cv::cvtColor(src, srcLab, CV_BGR2Lab);

	vector<cv::Mat> srcYCrCb_channels, srcLab_channels;
	cv::split(srcYCrCb, srcYCrCb_channels);
	cv::split(srcLab, srcLab_channels);
	cv::Mat skin_mask = ((srcYCrCb_channels[1] > 140) & (srcYCrCb_channels[1] < 159) & (srcYCrCb_channels[2] > 85) & (srcYCrCb_channels[2] < 135)) |
			((srcLab_channels[1] > 130) & (srcLab_channels[1] < 150) & (srcLab_channels[2] > 130));
	//cv::imshow("skin_mask", skin_mask);

	cv::Mat skin_mask_morph = skin_mask.clone();
	cv::dilate(skin_mask_morph, skin_mask_morph, cv::Mat(), cv::Point(-1, -1), 1);
	cv::erode(skin_mask_morph, skin_mask_morph, cv::Mat(), cv::Point(-1, -1), 4);
	cv::dilate(skin_mask_morph, skin_mask_morph, cv::Mat(), cv::Point(-1, -1), 4);
	//cv::imshow("skin_mask_morph", skin_mask_morph);

	cv::Mat large_connected_components = connectedComponentsFilter(skin_mask_morph.clone(), 500, 2);
	//imshow("large_connected_components", large_connected_components);

	cv::multiply(skin_mask, large_connected_components, skin_mask, 1/255.0);
	//cv::imshow("skin_mask_mult", skin_mask);

	cv::dilate(skin_mask, skin_mask, cv::Mat(), cv::Point(-1, -1), 1);
	//cv::imshow("skin_mask_dilate", skin_mask);

	return skin_mask;
}
