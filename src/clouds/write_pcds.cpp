// http://www.pointclouds.org/documentation/tutorials/openni_grabber.php#openni-grabber

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <limits>
#include <sstream>
#include "signal.h"
#include <boost/program_options.hpp>
#include <boost/timer.hpp>
#include "comm/comm.h"

using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

boost::signals2::connection c;
bool doPause;
float delay;
boost::timer timur;
comm::Publisher* pcd_pub;


void PressEnterToContinue() {
  std::cout << "Press ENTER to continue... " << flush;
  std::cin.ignore( std::numeric_limits <std::streamsize> ::max(), '\n' );
}

void cloud_cb_ (const pcl::PointCloud<ColorPoint>::ConstPtr &cloud)
{
  if (doPause) {
    PressEnterToContinue();
  }

  if (doPause || timur.elapsed() > delay) {
    timur.restart();
    string fname = pcd_pub->next();
    pcl::io::savePCDFileBinary(fname, *cloud);
    cout << "wrote " << fname << endl;
  }

}
  
void run ()
{
  // create a new grabber for OpenNI devices
  pcl::Grabber* interface = new pcl::OpenNIGrabber();

  // make callback function from member function
  boost::function<void (const pcl::PointCloud<ColorPoint>::ConstPtr&)> f = boost::bind (&cloud_cb_,  _1);

  // connect callback function for desired signal. In this case its a point cloud with color values
  c = interface->registerCallback (f);

  // start receiving point clouds
  interface->start ();

  // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
  while (c.connected())
    sleep(1);

  // stop the grabber
  interface->stop ();
}







void disconnect(int x) {
  cout << "disconnecting...";
  c.disconnect();
  cout << "done" << endl;
}


int main(int ac, char* av[])

{
  float freq;
  timur.restart();
  po::options_description opts("Allowed options");
  opts.add_options()
    ("help", "produce help message")
    ("pause,p", po::value< bool >(&doPause)->default_value(false)->implicit_value(true),"prompt user before each img")
    ("freq,f", po::value< float >(&freq)->default_value(100.),"max frequency")
    ;

  po::positional_options_description p;
  // p.add("output-directory", 1);

  po::variables_map vm;        
  po::store(po::command_line_parser(ac, av)
	    .options(opts)
	    .positional(p)
	    .run()
	    , vm);
  if (vm.count("help")) {
    cout << "usage: record_pcds  [options]" << endl;
    cout << opts << endl;
    return 0;
  }
  po::notify(vm);    

  delay = 1/freq;
  pcd_pub = new comm::Publisher("pcds","pcd");
  signal(SIGINT, &disconnect);
  run ();
}
