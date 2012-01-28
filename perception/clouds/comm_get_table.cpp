#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include "geom.h"
#include <boost/foreach.hpp>
#include "get_table.h"
#include <comm/comm2.h>
#include <my_assert.h>

using namespace pcl;
using namespace std;
using namespace Eigen;


int main(int argc, char* argv[]) {
  PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
  setDataRoot();
  string pcdfile = Names("kinect","pcd").getCur().first.string();
  cout << "reading " << pcdfile << endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdfile, *cloud) == -1) {
    PCL_ERROR(("couldn't read file " + pcdfile + "\n").c_str());
    return -1;
  }
  else {
    cout << "read " << cloud->size() << " points" << endl;
  }
  vector<Vector3f> corners; 
  Vector3f normal;
  getTable(cloud,corners,normal);

  PointCloud<PointXYZ>::Ptr rectCloud(new PointCloud<PointXYZ>);
  BOOST_FOREACH(Vector3f w, corners) rectCloud->push_back(PointXYZ(w[0],w[1],w[2]));
  pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
  viewer.addPointCloud (cloud);
  viewer.addPolygon<PointXYZ>(rectCloud,0,255,0);
  viewer.spin();

  path outPath = onceFile("table_corners.txt");
  cout << "writing to " << outPath << endl;
  ofstream outFile(outPath.string().c_str());
  ASSERT(!outFile.fail());
  for (int i=0; i<4; i++) {
    for (int j=0; j<3; j++) outFile << corners[i][j] << " ";
    outFile << endl;
  }
  outFile.close();


}
