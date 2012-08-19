#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cv.h>
#include <opencv2/core/core.hpp>
#include <vector>

//the rows of features are the vectors of the data set to be compressed.
Eigen::MatrixXf compressPCA(const cv::PCA& pca, Eigen::MatrixXf features);
//Return the sub image of a window of size (2*i_range, 2*j_range) centered at (i_pixel, j_pixel)
//The returned sub matrix might not have the dimensions of (i_range, j_range) because the window is clamped at the edges
cv::Mat windowRange(const cv::Mat& image, int i_pixel, int j_pixel, int i_range, int j_range);

Eigen::Vector3f toEigenVectorImage(const cv::Vec3b& pixel);
cv::Vec3b toCVMatImage(const Eigen::Vector3f& pixel);
void fillBorder(cv::Mat image, int border_width=100, int reduce_width=20, int threshold=30);
//returns a matrix where the rows are the pixel locations in row major order and the columns are the color channels
Eigen::MatrixXf toEigenMatrixImage(const cv::Mat& image);
//The rows of m are the pixel locations in row major order and the columns of m are the color channels
// if rows and cols are 0 (the default), then m is converted into a 1-dimensional horizontal image
cv::Mat toCVMatImage(const Eigen::MatrixXf& m, int rows=0, int cols=0);
//Every row of m is a pixel and every column of m is a color channel
//type is the openCV color transformation types as used in cvtColor (i.e. CV_BGR2Lab)
Eigen::MatrixXf colorTransform(const Eigen::MatrixXf& m, int type);

cv::Mat rotate90(cv::Mat src);
//Returns the n*90deg version of image_rot that matches closest to image_ref. The dimensions of the input images and the returned image are the same.
cv::Mat matchRotation(cv::Mat rot_image, cv::Mat ref_image);

// Wrapper for OpenCV's polylines. This one takes a vector instead of an array.
void polylines(cv::Mat im, std::vector<std::vector<cv::Point2f> > points, bool isClosed, const cv::Scalar & color, int thickness= 1, int lineType=8, int shift=0);
// Given a binary image, finds the corners of the biggest polygon contour.
std::vector<cv::Point2f> polyCorners(cv::Mat mask);
// Returns the corners of the rectangle
std::vector<cv::Point2f> rectCorners(cv::RotatedRect rect);
