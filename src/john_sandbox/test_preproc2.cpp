#include "clouds/filtering.h"
#include "clouds/cloud_ops.h"
#include "clouds/geom.h"
#include "clouds/get_table.h"
#include "clouds/preprocessing.h"
#include "clouds/utils_pcl.h"
#include "clouds/comm_pcl.h"
#include "utils/vector_io.h"
#include "perception/utils_perception.h"
#include "perception/plotting_perception.h"
#include "comm/comm.h"
#include "perception/utils_perception.h"
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/timer.hpp>
#include <boost/lexical_cast.hpp>
#include <pcl/io/pcd_io.h>
#include "simulation/simplescene.h"

using namespace pcl;
using namespace std;
using namespace Eigen;

extern Affine3f getCamToWorldFromTable(const vector<Vector3f>& corners);
extern VectorXi getLabels(ColorCloudPtr cloud, const MatrixXf& coeffs, const VectorXf& intercepts);
extern double timeOfDay();

int main(int argc, char* argv[]) {

  initComm();
  Eigen::internal::setNbThreads(2);
  FileSubscriber kinectSub("kinect","pcd");
  CloudMessage cloudMsg;

  kinectSub.recv(cloudMsg);
  ColorCloudPtr cloud = cloudMsg.m_data;
  vector<Vector3f> corners; 
  Vector3f normal;
  getTable(cloud,corners,normal);

  MatrixXf corners1(4,3);
  for (int i=0; i < 4; i++) corners1.row(i) = corners[i];
  Affine3f camToWorld = getCamToWorldFromTable(corners);

  vector< vector<float> > coeffs1 = floatMatFromFile("/home/joschu/python/image_proc/pixel_classifiers/rope/coeffs.txt");
  vector<float> intercepts1 = floatVecFromFile("/home/joschu/python/image_proc/pixel_classifiers/rope/intercepts.txt");



  MatrixXf coeffs = toEigenMatrix(coeffs1);
  VectorXf intercepts = toVectorXf(intercepts1);

  Scene scene;
  PointCloudPlot::Ptr plotPoints(new PointCloudPlot());
  scene.env->add(plotPoints);
  scene.startViewer();

  double tStart = timeOfDay();
  int n=0;
  while (kinectSub.recv(cloudMsg)) {
    ColorCloudPtr cloud = cloudMsg.m_data;
    cout << "cloud size " << cloud->width << cloud->height << endl;
    VectorXb mask = getPointsOnTable(cloud, camToWorld, corners1,-.01,1);
    cout << "mask: " << mask.rows() << " " << mask.cols() << endl;
    ColorCloudPtr cloudOnTable = maskCloud(cloud, mask);

    VectorXi labels = getLabels(cloudOnTable, coeffs, intercepts);
    ColorCloudPtr ropeCloud = maskCloud(cloudOnTable, labels.array()==boost::lexical_cast<int>(argv[1]));
    ColorCloudPtr downedRopeCloud = removeOutliers(downsampleCloud(ropeCloud,.01));

    plotPoints->setPoints1(downedRopeCloud);
    scene.step(0);

    n++;
  }

  cout << (timeOfDay() - tStart)/n << " per image" << endl;



  // viewer.addPointCloud (cloudOnTable);
  // viewer.addPolygon<PointXYZ>(rectCloud,255,0,0);
  // viewer.spin();


}
