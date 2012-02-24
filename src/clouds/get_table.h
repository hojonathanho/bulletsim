#include <vector>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

void getTable(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Eigen::Vector3f>& corners, Eigen::Vector3f& normals, int ind=0);
