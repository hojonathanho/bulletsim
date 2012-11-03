#include "cvmat.h"
#include <algorithm>

using namespace Eigen;
using namespace std;

Eigen::MatrixXf compressPCA(const cv::PCA& pca, Eigen::MatrixXf features) {
	int maxComponents = pca.eigenvectors.rows;
	cv::Mat cv_features(features.rows(), features.cols(), CV_32FC1, features.data());
	cv::Mat cv_comp_features(features.rows(), maxComponents, CV_32FC1);
	for( int i = 0; i < cv_comp_features.rows; i++ ) {
		pca.project(cv_features.row(i), cv_comp_features.row(i));
	}
	return Map<MatrixXf>((float*)cv_comp_features.data, cv_comp_features.rows, cv_comp_features.cols);
}

cv::Mat windowRange(const cv::Mat& image, int i_pixel, int j_pixel, int i_range, int j_range) {
	return image(cv::Range(max(i_pixel - i_range, 0), min(i_pixel + i_range, image.rows-1)),
			cv::Range(max(j_pixel - j_range, 0), min(j_pixel + j_range, image.cols-1)));
}

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

cv::Mat rotate90(cv::Mat src) {
	cv::Point2f srcTri[3], dstTri[3];

	// Compute warp matrix
	srcTri[0].x = 0;
	srcTri[0].y = 0;
	srcTri[1].x = src.cols - 1;
	srcTri[1].y = 0;
	srcTri[2].x = 0;
	srcTri[2].y = src.rows - 1;

	dstTri[0].x = 0;
	dstTri[0].y = src.rows - 1;
	dstTri[1].x = 0;
	dstTri[1].y = 0;
	dstTri[2].x = src.cols - 1;
	dstTri[2].y = src.rows - 1;

	cv::Mat warp_mat = cv::getAffineTransform( srcTri, dstTri );
	cv::Mat dst;
	cv::warpAffine( src, dst, warp_mat, cv::Size(src.cols, src.rows) );
	return dst;
}

//Returns the n*90deg version of image_rot that matches closest to image_ref. The dimensions of the input images and the returned image are the same.
cv::Mat matchRotation(cv::Mat rot_image, cv::Mat ref_image) {
	cv::Mat diff_image;
	cv::Mat match_image = rot_image;
	cv::absdiff(ref_image, rot_image, diff_image);
	float min_sim_score = toEigenMatrixImage(diff_image).rowwise().norm().sum();
	for (int i=0; i<3; i++) {
		rot_image = rotate90(rot_image);
		cv::absdiff(ref_image, rot_image, diff_image);
		float sim_score = toEigenMatrixImage(diff_image).rowwise().norm().sum();
		if (sim_score < min_sim_score) {
			match_image = rot_image;
			min_sim_score = sim_score;
		}
	}
	return match_image;
}

// Wrapper for OpenCV's polylines. This one takes a vector instead of an array.
void polylines(cv::Mat im, vector<vector<cv::Point2f> > points, bool isClosed, const cv::Scalar & color, int thickness, int lineType, int shift){
	cv::Point ** pts = new cv::Point*[points.size()];
	for (int ctr_id=0; ctr_id<points.size(); ctr_id++) {
		pts[ctr_id] = new cv::Point[points[ctr_id].size()];
		for (int v_id=0; v_id<points[ctr_id].size(); v_id++) {
			pts[ctr_id][v_id] = points[ctr_id][v_id];
		}
	}

	int * npts = new int[points.size()];

	// find number of points for each contour
	for (int i = 0; i < points.size() ; i++)
		npts[i] = points[i].size();

	// draw
	cv::polylines(im, const_cast<const cv::Point**>(pts), npts, points.size(), isClosed, color, thickness, lineType, shift);

	for (int ctr_id=0; ctr_id<points.size(); ctr_id++) delete [] pts[ctr_id];
	delete [] pts;
	delete [] npts;
}

// Given a binary image, finds the corners of the biggest polygon contour.
vector<cv::Point2f> polyCorners(cv::Mat mask) {
	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;

	cv::findContours( mask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

	assert(contours.size() != 0);

	// iterate through all the top-level contours,
	int idx = 0;
	int idx_max = 0;
	double area_max = cv::contourArea(cv::Mat(contours[idx_max]));
	for( ; idx >= 0; idx = hierarchy[idx][0] ) {
		double area = cv::contourArea(cv::Mat(contours[idx]));
		if (area > area_max) {
			idx_max = idx;
			area_max = area;
		}
	}

	cv::Mat curve(contours[idx_max]);
	curve.convertTo(curve, CV_32FC1);
	vector<cv::Point2f> poly_points;
	cv::approxPolyDP(curve, poly_points, 10, true);
	return poly_points;
}

// Returns the corners of the rectangle
vector<cv::Point2f> rectCorners(cv::RotatedRect rect) {
	vector<cv::Point2f> corners;

	vector<cv::Point2f> unrot_corners;
	unrot_corners.push_back(cv::Point2f(-rect.size.width/2.0, -rect.size.height/2.0));
	unrot_corners.push_back(cv::Point2f(-rect.size.width/2.0, rect.size.height/2.0));
	unrot_corners.push_back(cv::Point2f(rect.size.width/2.0, rect.size.height/2.0));
	unrot_corners.push_back(cv::Point2f(rect.size.width/2.0, -rect.size.height/2.0));

	float angle = rect.angle * M_PI/180.0;
	cv::Mat rot(2,2,CV_32FC1);
	rot.at<float>(0,0) = cos(angle);
	rot.at<float>(0,1) = -sin(angle);
	rot.at<float>(1,0) = sin(angle);
	rot.at<float>(1,1) = cos(angle);

	for (int i=0; i<unrot_corners.size(); i++) {
		cv::Mat corner = cv::Mat(rect.center) + rot*cv::Mat(unrot_corners[i]);
		corners.push_back(cv::Point2f(corner.at<float>(0,0), corner.at<float>(0,1)));
	}
	return corners;
}
