#include "cloud_filtering.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include "utils/my_exceptions.h"
#include "utils_pcl.h"
using namespace std;

int main() {

  ColorCloudPtr fullCloud = readPCD("/home/joschu/Data/comm_towel/kinect/data000000000000.pcd");
  
  cv::Mat labelImg = cv::imread("/home/joschu/Data/comm_towel/jpgs2/data000000000000.jpg.label.png");

  vector<cv::Mat> channels;
  cv::split(labelImg,channels);
  cv::Mat mask = channels[0] == 1;

  ColorCloudPtr towelCloud = maskCloud(fullCloud, mask);

  pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
  viewer.addPointCloud (towelCloud);
  viewer.spin();

}
