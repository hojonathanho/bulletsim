#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include "geom.h"
#include "get_table.h"
#include "comm/comm.h"
#include "utils/my_assert.h"

using namespace pcl;
using namespace std;
using namespace Eigen;
namespace po = boost::program_options;

int main(int argc, char* argv[]) {

  string infile="";
  int skip=0;

  po::options_description opts("Allowed options");
  opts.add_options()
    ("help,h", "produce help message")
    ("infile,i", po::value< string >(&infile),"input file")
    ("skip,s", po::value< int >(&skip),"skip");
  po::variables_map vm;        
  po::store(po::command_line_parser(argc, argv)
	    .options(opts)
	    .run()
	    , vm);
  if (vm.count("help")) {
    cout << "usage: comm_get_table [options]" << endl;
    cout << opts << endl;
    return 0;
  }
  po::notify(vm);

  initComm();



  PointCloud<PointXYZRGBA>::Ptr cloud (new PointCloud<PointXYZRGBA>);
  if (infile.size() == 0)
    infile = Names("kinect","pcd").getCur().first.string();
  cout << "reading " << infile << endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (infile, *cloud) == -1) {
    PCL_ERROR(("couldn't read file " + infile + "\n").c_str());
    return -1;
  }
  else {
    cout << "read " << cloud->size() << " points" << endl;
  }
  vector<Vector3f> corners; 
  Vector3f normal;
  getTable(cloud,corners,normal,skip);

  PointCloud<PointXYZ>::Ptr rectCloud(new PointCloud<PointXYZ>);
  BOOST_FOREACH(Vector3f w, corners) rectCloud->push_back(PointXYZ(w[0],w[1],w[2]));
  pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
  viewer.addPointCloud (cloud);
  viewer.addPolygon<PointXYZ>(rectCloud,0,255,0);
  viewer.spin();

  path outPath = onceFile("table_corners.txt");
  cout << "writing to " << outPath << endl;
  ofstream outFile(outPath.string().c_str());
  ENSURE(!outFile.fail());
  for (int i=0; i<4; i++) {
    for (int j=0; j<3; j++) outFile << corners[i][j] << " ";
    outFile << endl;
  }
  outFile.close();


}
