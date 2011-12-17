// from osg galaxy example

#include "plotting.h"
#include "simplescene.h"
#include "userconfig.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using boost::shared_ptr;
#include <iostream>
using namespace std;

int main(int argc, char* argv[]) {
  CFG->read(argc, argv);
  CFG->scene.enableIK = CFG->scene.enableHaptics = CFG->scene.enableRobot = false;
  Scene s;

  if (argc < 2) {
    cout << "must supply an argument 1,2,3,..." << endl;
    return 0;
  }

  if (!strcmp(argv[1],"1")) {
    shared_ptr<PlotPoints> plot1(new PlotPoints(20));
    vector<btVector3> pts;
    pts.push_back(btVector3(0,0,0));
    pts.push_back(btVector3(1,0,0));
    pts.push_back(btVector3(0,1,0));
    pts.push_back(btVector3(1,1,0));
    pts.push_back(btVector3(0,0,1));
    pts.push_back(btVector3(1,0,1));
    pts.push_back(btVector3(0,1,1));
    pts.push_back(btVector3(1,1,1));
    plot1->setPoints(pts);
    s.env->add(plot1);

  }  


  if (!strcmp(argv[1],"2")) {
    shared_ptr<PlotLines> plot2(new PlotLines());
    vector<btVector3> pts1;
    pts1.push_back(btVector3(0,0,0));
    pts1.push_back(btVector3(1,0,0));
    pts1.push_back(btVector3(0,1,0));
    pts1.push_back(btVector3(1,1,0));
    pts1.push_back(btVector3(0,0,1));
    pts1.push_back(btVector3(1,0,1));
    pts1.push_back(btVector3(0,1,1));
    pts1.push_back(btVector3(1,1,1));

    vector<btVector3> pts2;
    btVector3 v(.1,.1,.1);
    pts2.push_back(btVector3(0,0,0)+v);
    pts2.push_back(btVector3(1,0,0)+v);
    pts2.push_back(btVector3(0,1,0)+v);
    pts2.push_back(btVector3(1,1,0)+v);
    pts2.push_back(btVector3(0,0,1)+v);
    pts2.push_back(btVector3(1,0,1)+v);
    pts2.push_back(btVector3(0,1,1)+v);
    pts2.push_back(btVector3(1,1,1)+v);
    plot2->setPoints(pts1,pts2);

    s.env->add(plot2);


  }  



  else if (!strcmp(argv[1],"3")) {
    shared_ptr<PlotPoints> plot3(new PlotPoints());
    const string pcdfile = "../data/0003.pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdfile, *cloud) == -1) {
      PCL_ERROR(("couldn't read file " + pcdfile + "\n").c_str());
      return -1;
    }
    plot3->setPoints(cloud);
    s.env->add(plot3);
  }



  s.startViewer();
  s.viewerLoop();
}
