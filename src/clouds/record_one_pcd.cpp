// http://www.pointclouds.org/documentation/tutorials/openni_grabber.php#openni-grabber

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <sstream>
#include "signal.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

boost::signals2::connection c;
int counter, divider;
string outfile;

string& ensureSlash(string& s) {
  if (s[s.size()-1] != '/') s += '/';
  return s;
}



void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
  static double last = pcl::getTime ();
  if (counter == 0) {
    pcl::io::savePCDFileBinary(outfile, *cloud);
    cout << "wrote " << counter << endl;
    counter++;
    c.disconnect();
  }

    
}
  
void run ()
{
  // create a new grabber for OpenNI devices
  pcl::Grabber* interface = new pcl::OpenNIGrabber();

  // make callback function from member function
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
    boost::bind (&cloud_cb_,  _1);

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
  counter=0;

  po::options_description opts("Allowed options");
  opts.add_options()
    ("help", "produce help message")
    ("outfile,o", po::value< string >(&outfile), "oubput directory")
    ;

  po::positional_options_description p;
  p.add("outfile", 1);

  po::variables_map vm;        
  po::store(po::command_line_parser(ac, av)
	    .options(opts)
	    .positional(p)
	    .run()
	    , vm);



  if (vm.count("help")) {
    cout << "usage: record_one_pcd file" << endl;
    cout << opts << endl;
    return 0;
  }
  po::notify(vm);    
  run ();
}
