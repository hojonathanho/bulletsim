#include "utils_pcl.h"
#include <pcl/compression/octree_pointcloud_compression.h>
#include <fstream>
#include <iostream>
using namespace std;
using namespace pcl;

int main() {
  ifstream infile("/home/joschu/Data/pcl/test.pcc");
  pcl::octree::PointCloudCompression<ColorPoint>* PointCloudDecoder = new pcl::octree::PointCloudCompression<ColorPoint> ();
  ColorCloudPtr cloud(new ColorCloud());
  sleep(1);
  PointCloudDecoder->decodePointCloud (infile, cloud);
  cout << "time stamp: " << cloud->header.stamp << endl;
}
