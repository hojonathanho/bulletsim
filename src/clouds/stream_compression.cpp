#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/compression/octree_pointcloud_compression.h>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <stdio.h>
#include <fstream>
#include <stdlib.h>
#include <signal.h>
#include <sys/time.h>
#include <string>


#ifdef WIN32
# define sleep(x) Sleep((x)*1000)
#endif

bool wantsExit = false;
void setWantsExit(int) {
  wantsExit = true;
}


class CloudCompressor
{

	ofstream m_cloudFile;
	ofstream m_stampFile;

public:
  CloudCompressor (fs::path cloudPath) :
	  m_cloudFile(cloudPath.c_str()),
	  m_stampFile((cloudPath.string() + std::string(".txt")).c_str())
  {
	  m_stampFile << setprecision(15);
  }

  void
  cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
  {
      // stringstream to store compressed point cloud
      // output pointcloud
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGBA> ());

      // compress point cloud
      timeval tim;
      gettimeofday(&tim, NULL);
      double time = tim.tv_sec+(tim.tv_usec/1000000.0);


      PointCloudEncoder->encodePointCloud (cloud, m_cloudFile);

      m_stampFile << time << endl;
  }

  void
  run ()
  {

    bool showStatistics = true;

    // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
    pcl::octree::compression_Profiles_e compressionProfile = pcl::octree::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

    // instantiate point cloud compression for encoding and decoding
    PointCloudEncoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGBA> (compressionProfile, showStatistics);

    // create a new grabber for OpenNI devices
    pcl::Grabber* interface = new pcl::OpenNIGrabber ();

    // make callback function from member function
    boost::function<void
    (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&CloudCompressor::cloud_cb_, this, _1);

    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback (f);

    // start receiving point clouds
    interface->start ();

    while (!wantsExit)
    {
      sleep (1);
    }

    interface->stop ();

    // delete point cloud compression instances
    delete (PointCloudEncoder);

  }


  pcl::octree::PointCloudCompression<pcl::PointXYZRGBA>* PointCloudEncoder;

};

int
main (int argc, char **argv)
{
  CloudCompressor v("/home/joschu/Data/pcl/test.pcc");
  v.run ();

  return (0);
}
