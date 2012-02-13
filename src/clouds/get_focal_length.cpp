 #include <pcl/io/openni_grabber.h>
 #include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <iostream>
using namespace std;
 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

     void cloud_cb_ (const boost::shared_ptr<openni_wrapper::DepthImage> di)
     {
       cout << di->getFocalLength() << endl;

     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback (f);

       interface->start ();

       while (!viewer.wasStopped())
       {
         sleep (1);
       }

       interface->stop ();
     }

     pcl::visualization::CloudViewer viewer;
 };

 int main ()
 {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }
