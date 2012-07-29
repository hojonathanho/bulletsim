#include "utils_cv.h"
#include <vector>
#include <iostream>
using namespace std;

cv::Mat toSingleChannel(const cv::Mat& in) {
  vector<cv::Mat> channels;
  cv::split(in, channels);
  return channels[0];
}

// input and output images are binaries (i.e 0's and 255's). src gets modified.
cv::Mat connectedComponentsFilter(cv::Mat src, int min_pix) {
	cv::Mat dst = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);

	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;

	cv::findContours( src, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

	// iterate through all the top-level contours,
	// draw each connected component with its own random color
	int idx = 0;
	for( ; idx >= 0; idx = hierarchy[idx][0] )
	{
		double area = cv::contourArea(cv::Mat(contours[idx]));
		cv::Scalar color;
		if (area > min_pix) color = cv::Scalar(255);
		else color = cv::Scalar(0);
		cv::drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy );
	}
	return dst;
}

// input is an bgr image and output is a binary mask (i.e. 0's are likely to not be skin and 255's are likely to be skin). src doesn't get modified.
// YCrCb thresholds from http://waset.org/journals/waset/v43/v43-91.pdf except for Ymin = 40
// merging from here http://www.csee.wvu.edu/~richas/papers/tkjse.pdf (?)
cv::Mat skinMask(cv::Mat src) {
	cv::Mat srcYCrCb(src.rows, src.cols, CV_8UC3);
	cv::cvtColor(src, srcYCrCb, CV_BGR2YCrCb);
	cv::Mat srcLab(src.rows, src.cols, CV_8UC3);
	cv::cvtColor(src, srcLab, CV_BGR2Lab);

	cv::Mat skin_mask = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
	for (int i=0; i<src.rows; i++) {
		for (int j=0; j<src.cols; j++) {
			cv::Vec3b YCrCb = srcYCrCb.at<cv::Vec3b>(i,j);
			cv::Vec3b Lab = srcLab.at<cv::Vec3b>(i,j);
				if (	((YCrCb[0] > 80 &&
						YCrCb[1] > 135 && YCrCb[1] < 180 &&
						YCrCb[2] > 85 && YCrCb[2] < 135) ||
						(Lab[1] > 130 && Lab[1] < 150 &&
								Lab[2] > 130)) ) {
				skin_mask.at<char>(i,j) = 255;
			}
		}
	}
	//cv::imshow("skin_mask", skin_mask);

	cv::Mat skin_mask_morph = skin_mask.clone();
	cv::dilate(skin_mask_morph, skin_mask_morph, cv::Mat(), cv::Point(-1, -1), 1);
	cv::erode(skin_mask_morph, skin_mask_morph, cv::Mat(), cv::Point(-1, -1), 3);
	cv::dilate(skin_mask_morph, skin_mask_morph, cv::Mat(), cv::Point(-1, -1), 4);
	//cv::imshow("skin_mask_morph", skin_mask_morph);

	cv::Mat large_connected_components = connectedComponentsFilter(skin_mask_morph.clone(), 1000);
	//imshow("large_connected_components", large_connected_components);

	cv::multiply(skin_mask, large_connected_components, skin_mask, 1/255.0);
	//cv::imshow("skin_mask_mult", skin_mask);

	cv::dilate(skin_mask, skin_mask, cv::Mat(), cv::Point(-1, -1), 2);
	//cv::imshow("skin_mask_dilate", skin_mask);

	return skin_mask;
}
