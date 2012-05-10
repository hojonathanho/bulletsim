#include <iostream>
#include "clouds/utils_pcl.h"
#include <pcl/io/pcd_io.h>
using namespace std;

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr convert(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr in) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
  out->points.resize(in->size());
  for (size_t i = 0; i < in->size(); ++i) {
    out->points[i].x = in->points[i].x;
    out->points[i].y = in->points[i].y;
    out->points[i].z = in->points[i].z;
    out->points[i].r = in->points[i].r;
    out->points[i].g = in->points[i].g;
    out->points[i].b = in->points[i].b;
  }
  return out;
}

int main (int argc, char** argv) {
  if (argc != 3) {
    cerr << "usage: " << argv[0] << " in.pcd out.pcd" << endl;
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr in(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argv[1], *in);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out = convert(in);
  pcl::io::savePCDFileBinary(argv[2], *out);

  return 0;
}
