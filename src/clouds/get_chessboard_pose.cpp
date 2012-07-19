#include "clouds/get_chessboard_pose.h"

using namespace cv;
using namespace Eigen;

//the transform brings points from the camera coordinate system to the reference (center of chess board) coordinate system
bool get_chessboard_pose(Mat& image, int width_cb, int height_cb, double size, const Matrix3f& cam_matrix, Matrix4f& transform) {
	Mat gray_image;
	cvtColor(image, gray_image, CV_BGR2GRAY);

	vector<Point2f> corners;
	vector<Point3f> obj(width_cb*height_cb);
	for (int i=0; i<height_cb; i++) {
		for (int j=0; j<width_cb; j++) {
			float x = (j - ((float) width_cb - 1.0)/2.0);
			float y = (i - ((float) height_cb - 1.0)/2.0);
			obj[i*width_cb+j] = size*Point3f(x, y, 0.0);
		}
	}

	bool found = findChessboardCorners(gray_image, Size(width_cb,height_cb), corners);
	// CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
	if(found) {
		cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

		drawChessboardCorners(image, Size(width_cb,height_cb), corners, found);
		imshow("win", image);
		/*
		int key = waitKey(1);
		while (key != ' ') {
			imshow("win", image);
		  key = waitKey(1);
		}
		*/
	} else {
		return false;
	}

	double _intrinsic[9] = { cam_matrix(0,0), cam_matrix(0,1), cam_matrix(0,2),
							 cam_matrix(1,0), cam_matrix(1,1), cam_matrix(1,2),
						     cam_matrix(2,0), cam_matrix(2,1), cam_matrix(2,2) };
	Mat intrinsic(3, 3, CV_64F, _intrinsic);
	double _distCoeffs[5] = {0,0,0,0,0};
	Mat distCoeffs(1,5, CV_64F, _distCoeffs);

	Mat rvec, tvec;
	solvePnP(obj, corners, intrinsic, distCoeffs, rvec, tvec);

	Vector3f translation;
	Matrix3f rotation;
	MatConstIterator_<double> it = tvec.begin<double>();
	MatConstIterator_<double> it_end = tvec.end<double>();
	for (int i=0; i<3; i++) {
		translation(i) = *it;
		it++;
	}
	Mat rot;
	Rodrigues(rvec, rot);
	it = rot.begin<double>();
	it_end = rot.end<double>();
	for (int i=0; i<3; i++) {
		for (int j=0; j<3; j++) {
			rotation(i,j) = *it;
			it++;
		}
	}

	//brings points from the model coordinate system to the camera coordinate system
	Matrix4f modelToCameraTransform;
	modelToCameraTransform.block(0,0,3,3) = rotation;
	modelToCameraTransform.block(3,0,1,4) = Vector4f(0,0,0,1).transpose();
	modelToCameraTransform.block(0,3,3,1) = translation;

	//brings points from the camera coordinate system to the model coordinate system
	transform = modelToCameraTransform.inverse();

	// There are two possible transforms. One of them represents when the camera looks
	// at the chess board from above it, and the other one when the camera is at the
	// mirror position and orientation with respect to the chess board plane.
	// If the z coordinate (transform(3,2)) of the camera is positive, then the transform
	// represents when the camera is looks from above.
	if (transform(2,3) < 0)
		transform = Vector4f(-1,1,-1,1).asDiagonal() * transform;

	return true;
}
