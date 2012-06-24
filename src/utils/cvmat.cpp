#include "cvmat.h"

using namespace Eigen;

//As usual, cv::Mat is CV_8UC3 BGR
//Vector3f mean(const cv::Mat& image) {
//	Vector3f rgb(0,0,0);
//	int rgb_ct = 0;
//	for (int y=0; y<image.rows; y++) {
//		for (int x=0; x<image.cols; x++) {
//			if (image.at<cv::Vec3b>(y,x) != cv::Vec3b(0,0,0)) {
//				for (int c=0; c<3; c++)
//					rgb(c) += (int) image.at<cv::Vec3b>(y,x)[2-c];
//				rgb_ct++;
//			}
//		}
//	}
//	if (rgb_ct != 0) rgb /= rgb_ct;
//	return rgb;
//}

//If image.type() == CV_8UC3, returns a matrix where the rows are the pixel locations in row major order and the columns are the color channels
MatrixXf toEigenMatrix(const cv::Mat& image) {
	if (image.type() != CV_32FC1 && image.type() != CV_8UC3)
		throw std::runtime_error("input matrix has the wrong type");
	if (image.type() == CV_32FC1)
		return Map<MatrixXf>((float*)image.data, image.rows, image.cols);
	MatrixXf m(image.cols*image.rows, 3);
	for (int i=0; i<image.rows; i++)
		for (int j=0; j<image.cols; j++)
			for (int c=0; c<3; c++)
				m(i*image.cols+j,c) = ((float) image.at<cv::Vec3b>(i,j)[c]) / 255.0;
	return m;
}

//The rows of m are the pixel locations in row major order and the columns of m are the color channels
// if rows and cols are 0 (the default), then m is converted into a 1-dimensional horizontal image
cv::Mat toCVMat(const MatrixXf& m, int rows, int cols) {
	if (rows == 0 && cols == 0) {
		rows = 1;
		cols = m.rows();
	}
	assert(m.cols() == 3);
	assert(m.rows() == rows*cols);
	cv::Mat image(rows, cols, CV_8UC3);
  for (int i=0; i<image.rows; i++)
	  for (int j=0; j<image.cols; j++)
	  	for (int c=0; c<3; c++)
  			image.at<cv::Vec3b>(i,j)[c] = (unsigned char) (m(i*image.cols+j,c) * 255.0);
  return image;
}

//Every row of m is a pixel and every column of m is a color channel
//type is the openCV color transformation types as used in cvtColor (i.e. CV_BGR2Lab)
MatrixXf colorTransform(const MatrixXf& m, int type) {
	assert(m.cols() == 3);
	cv::Mat image = toCVMat(m);
	cvtColor(image, image, type);
	return toEigenMatrix(image);
}
