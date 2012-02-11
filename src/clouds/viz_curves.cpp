#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include "utils_pcl.h"
#include "utils/vector_io.h"


using namespace pcl;
using namespace std;
namespace po = boost::program_options;


int main(int argc, char* argv[]) {

  vector<string> pcdfiles;
  vector<string> curvefiles;

  po::options_description opts("Allowed options");
  opts.add_options()
    ("help,h", "produce help message")
    ("point_clouds,p", po::value< vector<string> > (&pcdfiles))
    ("curves,c", po::value< vector<string> > (&curvefiles));
  po::variables_map vm;        
  po::store(po::command_line_parser(argc, argv)
	    .options(opts)
	    .run()
	    , vm);
  if (vm.count("help")) {
    cout << "usage: comm_downsample_clouds [options]" << endl;
    cout << opts << endl;
    return 0;
  }
  po::notify(vm);

  pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");

  BOOST_FOREACH(string& pcdfile, pcdfiles) {
    ColorCloudPtr cloud = readPCD(pcdfile);
    viewer.addPointCloud(cloud);
  }

  BOOST_FOREACH(string& curvefile, curvefiles) {
    vector< vector<float> > points = floatMatFromFile(curvefile);
    PointCloud<PointXYZ>::Ptr cloud = toPointCloud(points);
    cout << points << endl;
    viewer.addPolygon<PointXYZ>(cloud,0,255,0);
  }

  viewer.spin();

}
