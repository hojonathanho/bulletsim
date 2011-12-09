#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include "geom.h"
#include "get_table.h"

using namespace pcl;
using namespace std;
using namespace Eigen;


int main() {
  PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
  const string pcdfile = "/home/joschu/Data/comm/pcds/msg000000000000.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdfile, *cloud) == -1) {
    PCL_ERROR(("couldn't read file " + pcdfile + "\n").c_str());
    return -1;
  }
  vector<Vector3f> corners; 
  Vector3f normal;
  getTable(cloud,corners,normal);

  cout << "corners:" << endl;
  BOOST_FOREACH(Vector3f c, corners) cout << c;
  cout << endl;


  PointCloud<PointXYZ>::Ptr rectCloud(new PointCloud<PointXYZ>);
  BOOST_FOREACH(Vector3f w, corners) rectCloud->push_back(PointXYZ(w[0],w[1],w[2]));
  pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
  viewer.addPointCloud (cloud);
  viewer.addPolygon<PointXYZ>(rectCloud,255,0,0);
  viewer.spin();

}
