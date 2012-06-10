#include "sensor_msgs/Image.h"
#include "hydra_msgs/Calib.h"
#include <iostream>
#include <ros/ros.h>
using namespace std;


void callback(const hydra_msgs::Calib& msg) {
  cout << msg << endl;
}

int main(int argc, char* argv[]) {
  sensor_msgs::Image p;
  cout << p << endl;
  ros::NodeHandle nh;

  ros::init(argc, argv, "test_recv_hydra_msg");
  ros::Subscriber sub = nh.subscribe("hydra_calib", 1000, callback);
  ros::spin();
  return 0;
}
