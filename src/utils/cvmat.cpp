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

Vector3f toEigenVectorImage(const cv::Vec3b& pixel) {
	return Vector3f( ((float) pixel[0])/255.0, ((float) pixel[1])/255.0, ((float) pixel[2])/255.0 );
}

cv::Vec3b toCVMatImage(const Vector3f& pixel) {
	return cv::Vec3b( (uchar) (pixel(0)*255.0), (uchar) (pixel(1)*255.0), (uchar) (pixel(2)*255.0) );
}

void reduceVertical(cv::Mat src, cv::Mat dst) {
	assert(src.rows%dst.rows == 0);
	assert(src.cols == dst.cols);
	//average going down
	int i_stride = src.rows/dst.rows;
	for (int j=0; j<dst.cols; j++) {
		for (int i=0; i<dst.rows; i++) {
			Vector3f pixel = Vector3f::Zero();
			for (int i_src=i_stride*i; i_src<(i_stride*i+i_stride); i_src++)
				pixel += toEigenVectorImage(src.at<cv::Vec3b>(i_src,j));
			pixel /= (float) i_stride;
			dst.at<cv::Vec3b>(i,j) = toCVMatImage(pixel);
		}
	}
}

void reduceHorizontal(cv::Mat src, cv::Mat dst) {
	assert(src.rows == dst.rows);
	assert(src.cols%dst.cols == 0);
	//average going right
	int j_stride = src.cols/dst.cols;
	for (int i=0; i<dst.rows; i++) {
		for (int j=0; j<dst.cols; j++) {
			Vector3f pixel = Vector3f::Zero();
			for (int j_src=j_stride*j; j_src<(j_stride*j+j_stride); j_src++)
				pixel += toEigenVectorImage(src.at<cv::Vec3b>(i,j_src));
			pixel /= (float) j_stride;
			dst.at<cv::Vec3b>(i,j) = toCVMatImage(pixel);
		}
	}
}

//Reduces the image size from src to dst by averaging pixels in src that corresponds to a single pixel in dst.
//Requires that the dimensions of src is a multiple of dst.
void reduce(cv::Mat src, cv::Mat dst) {
	assert(src.rows%dst.rows == 0);
	assert(src.cols%dst.cols == 0);
	cv::Mat red_vert(dst.rows, src.cols, dst.type());
	reduceVertical(src, red_vert);
	reduceHorizontal(red_vert, dst);
}

//Fills (replaces) "hole" pixels from src with the pixel in dst at that same position.
//"Hole" pixels are the ones whose value is less than cv::Vec3b(threshold, threshold, threshold).
void fillHoles(cv::Mat src, cv::Mat dst, int threshold=20) {
	assert(src.rows%dst.rows == 0);
	assert(src.cols%dst.cols == 0);
	for (int i=0; i<dst.rows; i++)
		for (int j=0; j<dst.cols; j++)
			if (dst.at<cv::Vec3b>(i,j)[0] < threshold &&
				dst.at<cv::Vec3b>(i,j)[1] < threshold &&
				dst.at<cv::Vec3b>(i,j)[2] < threshold)
				dst.at<cv::Vec3b>(i,j) = src.at<cv::Vec3b>(i,j);
}

//Fills (replaces) "hole" pixels in image that are within border_width pixels from the image's border.
//"Hole" pixels are the ones whose value is less than cv::Vec3b(threshold, threshold, threshold).
//The "hole" pixels are replaced with an average of the inner reduce_width pixels in the same line as the "hole" pixel.
void fillBorder(cv::Mat image, int border_width, int reduce_width, int threshold) {
	for(int i=image.rows-1-border_width; i<(image.rows-1); i++) {
		cv::Mat fill_image(1, image.cols, image.type());
		reduce(image.rowRange(cv::Range(i-reduce_width+1, i+1)), fill_image);
		fillHoles(fill_image, image.row(i+1), threshold);
	}
	for(int i=border_width; i>0; i--) {
		cv::Mat fill_image(1, image.cols, image.type());
		reduce(image.rowRange(cv::Range(i, i+reduce_width)), fill_image);
		fillHoles(fill_image, image.row(i-1), threshold);
	}
	for(int j=image.cols-1-border_width; j<(image.cols-1); j++) {
		cv::Mat fill_image(image.rows, 1, image.type());
		reduce(image.colRange(cv::Range(j-reduce_width+1, j+1)), fill_image);
		fillHoles(fill_image, image.col(j+1), threshold);
	}
	for(int j=border_width; j>0; j--) {
		cv::Mat fill_image(image.rows, 1, image.type());
		fillHoles(fill_image, image.col(j-1), threshold);
	}
}

//If image.type() == CV_8UC3, returns a matrix where the rows are the pixel locations in row major order and the columns are the color channels
MatrixXf toEigenMatrixImage(const cv::Mat& image) {
	if (image.type() != CV_8UC3)
		throw std::runtime_error("input matrix has the wrong type");
	MatrixXf m(image.cols*image.rows, 3);
	for (int i=0; i<image.rows; i++)
		for (int j=0; j<image.cols; j++)
			for (int c=0; c<3; c++)
				m(i*image.cols+j,c) = ((float) image.at<cv::Vec3b>(i,j)[c]) / 255.0;
	return m;
}

//The rows of m are the pixel locations in row major order and the columns of m are the color channels
// if rows and cols are 0 (the default), then m is converted into a 1-dimensional horizontal image
cv::Mat toCVMatImage(const MatrixXf& m, int rows, int cols) {
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
	cv::Mat image = toCVMatImage(m);
	cvtColor(image, image, type);
	return toEigenMatrixImage(image);
}
