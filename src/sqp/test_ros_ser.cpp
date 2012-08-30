#include <iostream>
#include <std_msgs/String.h>
#include "utils_ser.h"
#include <ros/ros.h>
using namespace std;



int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_ros_ser");


  std_msgs::String msg;
  msg.data = "blahblahblah";
  msgToFile(msg, "/tmp/test.msg");


  std_msgs::String msg1 = msgFromFile<std_msgs::String>("/tmp/test.msg");
  cout << msg1 << endl;
//
//  uint32_t serial_size = 10000;//ros::serialization::serializationLength(msg);
//  printf("serial size: %i\n", serial_size);
//  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
//  ser::OStream stream(buffer.get(), serial_size);
//  ser::serialize(stream, msg);
//
//  cout << "buffer" << (char*)buffer.get() << endl;
//  for (int i=0; i < serial_size; ++i) cout << buffer[i] << " ";
//  cout << endl;
////  ifstream infile("/tmp/req.msg");
////
////
////  PlanTrajRequest req1;
////  uint32_t serial_size1 = 10000;//ser::serializationLength(req1);
////  boost::shared_array<uint8_t> buffer1(new uint8_t[serial_size1]);
////  infile >> buffer.get();
//  std_msgs::String msg1;
//  ser::IStream stream1(buffer.get(), serial_size);
//  ser::deserialize(stream1, msg1);
////
////
//  cout << msg1 << endl;
}
