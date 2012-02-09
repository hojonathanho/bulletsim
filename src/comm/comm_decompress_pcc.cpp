#include <iostream>
#include <string>
#include "comm/comm2.h"
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

  initComm();
  CloudGrabber pub(topic,doPause,downsample);
  pub.run();

}
