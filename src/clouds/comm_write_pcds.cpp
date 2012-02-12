#include <iostream>
#include <string>
#include "comm/comm.h"
#include "comm_pcl.h"
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;

int main(int argc, char* argv[]) {

  string topic;
  bool doPause;
  int downsample;

  po::options_description opts("Allowed options");
  opts.add_options()
    ("help,h", "produce help message")
    ("topic,t", po::value< string >(&topic)->default_value("kinect"),"topic name")
    ("pause,p", po::value< bool >(&doPause)->default_value(false)->implicit_value(true),"prompt user before each img")
    ("downsample,d", po::value< int >(&downsample)->default_value(1),"max frequency");
  po::variables_map vm;        
  po::store(po::command_line_parser(argc, argv)
	    .options(opts)
	    .run()
	    , vm);
  if (vm.count("help")) {
    cout << "usage: record_pcds  [options]" << endl;
    cout << opts << endl;
    return 0;
  }
  po::notify(vm);    

  setDataRoot();

  if (doPause) {
    PausingCloudGrabber pub(topic);
    pub.run();
  }
  else {
    CloudGrabber pub(topic,downsample);
    pub.run();
  }

}
