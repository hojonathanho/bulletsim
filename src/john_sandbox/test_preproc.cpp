#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include "clouds/geom.h"
#include "clouds/get_table.h"
#include "clouds/preprocessing.h"
#include "clouds/utils_pcl.h"
#include "clouds/filtering.h"
#include "clouds/cloud_ops.h"
#include <boost/timer.hpp>

using namespace pcl;
using namespace std;
using namespace Eigen;

extern Affine3f getCamToWorldFromTable(const vector<Vector3f>& corners);
extern VectorXi getLabels(ColorCloudPtr cloud, const MatrixXf& coeffs, const VectorXf& intercepts);
extern double timeOfDay();

int main() {
  ColorCloudPtr cloud = readPCD("/home/joschu/bulletsim/src/sandbox/towel.pcd");
  vector<Vector3f> corners; 
  Vector3f normal;
  getTable(cloud,corners,normal);

  MatrixXf corners1(4,3);
  for (int i=0; i < 4; i++) corners1.row(i) = corners[i];
  Affine3f camToWorld = getCamToWorldFromTable(corners);

  double tStart = timeOfDay();

  Eigen::internal::setNbThreads(2);

  for (int i=0; i < 100; i++) {
  ColorCloudPtr cloudOnTable;
  VectorXb mask = getPointsOnTable(cloud, camToWorld, corners1,0,1);
  cloudOnTable = maskCloud(cloud, mask);
  MatrixXf coeffs(6,7);
  VectorXf intercepts(7);
  coeffs.setRandom();
  intercepts.setZero();

  VectorXi labels;
  labels = getLabels(cloudOnTable, coeffs, intercepts);

  ColorCloudPtr towelCloud = maskCloud(cloudOnTable, labels.array()< 3);
  ColorCloudPtr ropeCloud = maskCloud(cloudOnTable, labels.array()>3);
  ColorCloudPtr downedTowelCloud = removeOutliers(downsampleCloud(towelCloud,.02));
  ColorCloudPtr downedRopeCloud = removeOutliers(downsampleCloud(ropeCloud,.01));

  }

  cout << (timeOfDay() - tStart)/100 << " per iteration" << endl;


  PointCloud<PointXYZ>::Ptr rectCloud(new PointCloud<PointXYZ>);
  BOOST_FOREACH(Vector3f w, corners) rectCloud->push_back(PointXYZ(w[0],w[1],w[2]));

  // pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
  // viewer.addPointCloud (cloudOnTable);
  // viewer.addPolygon<PointXYZ>(rectCloud,255,0,0);
  // viewer.spin();


}
