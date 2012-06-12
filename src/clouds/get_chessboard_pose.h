#include <vector>
#include <cv.h>
#include <highgui.h>
#include <Eigen/Geometry>

bool get_chessboard_pose(cv::Mat& image, int width_cb, int height_cb, double size, const Eigen::Matrix3f& cam_matrix, Eigen::Matrix4f& transform);
