#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "src/plotting.h"

class PointCloudPlot : public PlotPoints {
public:
  PointCloudPlot(float size=5) : PlotPoints(size) {}
  void setPoints1(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud);
};
