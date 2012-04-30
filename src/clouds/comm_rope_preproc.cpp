#include "utils/my_exceptions.h"
#include "clouds/preprocessing.h"
#include "clouds/comm_pcl.h"
#include "clouds/comm_cv.h"


using namespace std;

int main(int argc, char* argv[]) {

  // po::options_description opts("Allowed options");
  // opts.add_options()
  //   ("help,h", "produce help message")
  // po::variables_map vm;        
  // po::store(po::command_line_parser(argc, argv)
  // 	    .options(opts)
  // 	    .run()
  // 	    , vm);
  // if (vm.count("help")) {
  //   cout << "usage: comm_downsample_clouds [options]" << endl;
  //   cout << opts << endl;
  //   return 0;
  // }
  // po::notify(vm);
 
  initComm();
  extern bool LIVE;
  cout << "LIVE: " << LIVE << endl;
  CloudMessage inputCloudMsg;
  ImageMessage labelMsg;
  FileSubscriber cloudSub("kinect","pcd");
  FilePublisher cloudPub("rope_pts","pcd");

  RopePreprocessor tp;

  while (true) { 
    bool gotOne = cloudSub.recv(inputCloudMsg);
    cout << "GOTIT" << gotOne << endl;
    if (!gotOne) break;

    ColorCloudPtr outputCloud = tp.extractRopePoints(inputCloudMsg.m_data);
    cloudPub.send(CloudMessage(outputCloud));

  }


}
