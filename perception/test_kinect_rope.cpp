#include "userconfig.h"
#include "simplescene.h"
#include "util.h"
#include "rope.h"
#include "plotting.h"
#include "tracking.h"

#include "get_table.h"
#include "clouds/geom.h"
#include "utils_perception.h"
#include "comm.h"

#include "unistd.h"
#include <json/json.h>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
//#include "pcl/common/eigen.h"
#include <boost/foreach.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using boost::shared_ptr;
using namespace Eigen;
using namespace util;
// make table and rope from kinect data

float METERS;


btVector3 jsonVec(Json::Value v) {
  return btVector3(v[0u].asDouble(), v[1u].asDouble(), v[2u].asDouble());
}


struct EndInfo {
  btPoint2PointConstraint* ptp0;
  btPoint2PointConstraint* ptp1;
  bool active0;
  bool active1;
};

BulletObject::Ptr end0sphere;
BulletObject::Ptr end1sphere;

void updateEnds(const string jsonfile, btDynamicsWorld* world, EndInfo& endinfo, const btTransform& cam2world) {

  std::stringstream buffer;
  std::ifstream infile(jsonfile.c_str());
  assert(!infile.fail());
  buffer << infile.rdbuf();
  Json::Reader reader;
  Json::Value root;
  bool parsedSuccess = reader.parse(buffer.str(), root, false);
  assert(parsedSuccess);

  btVector3 pivot0 = (cam2world*jsonVec(root["front"]["xyz"]))*METERS;
  btVector3 pivot1 = (cam2world*jsonVec(root["back"]["xyz"]))*METERS;

  cout << "end constraints" << endl;
  btVector3 vec = pivot0;
  cout << vec.getX() QQ vec.getY() QQ vec.getZ() << endl;
  vec = pivot1;
  cout << vec.getX() QQ vec.getY() QQ vec.getZ() << endl;

  endinfo.ptp0->setPivotB(pivot0);
  endinfo.ptp1->setPivotB(pivot1);

  bool newActive0 = root["front"]["seen"].asBool();
  bool newActive1 = root["back"]["seen"].asBool();
  bool oldActive0 = endinfo.active0;
  bool oldActive1 = endinfo.active1;

  //hack to deal with case where we get an erroneous constraint

  if (pivot0.getZ()/METERS > 1.1 || !isfinite(pivot0.getZ())) newActive0 = false;
  if (pivot1.getZ()/METERS > 1.1 || !isfinite(pivot0.getZ())) newActive1 = false;

  cout << newActive0 QQ newActive1 QQ oldActive0 QQ oldActive1 << endl;

  if (newActive0 && !oldActive0) world->addConstraint(endinfo.ptp0);
  if (newActive1 && !oldActive1) world->addConstraint(endinfo.ptp1);
  if (!newActive0 && oldActive0) world->removeConstraint(endinfo.ptp0);
  if (!newActive1 && oldActive1) world->removeConstraint(endinfo.ptp1);

  endinfo.active0 = newActive0;
  endinfo.active1 = newActive1;

  end0sphere->rigidBody->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1),pivot0));
  end1sphere->rigidBody->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1),pivot1));
  if (newActive0) end0sphere->setColor(0,0,1,.25);
  else end0sphere->setColor(1,1,1,.1);
  if (newActive1) end1sphere->setColor(0,0,1,.25);
  else end1sphere->setColor(1,1,1,.1);


}



int main(int argc, char *argv[]) {
  setConfigData(CFG2);
  CFG->read(argc,argv);
  CFG->scene.enableIK = CFG->scene.enableHaptics = CFG->scene.enableRobot = false;


  METERS = CFG->scene.scale;
  cout << "scale: " << METERS << endl;

  string first_rope = comm::listenOnce("first_rope.txt");
  string first_ends = comm::listenOnce("first_ends.txt");

  comm::Listener rope_listener("rope");
  comm::Listener pcd_listener("pcds");
  comm::Listener ends_listener("ends");


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = readPCD(pcd_listener.next(MODE_NOUP));
  vector<Vector3f> corners_cam;
  Vector3f normal;
  getTable(cloud,corners_cam,normal);
  Affine3f cam2world = getCam2World(corners_cam,normal);
  btTransform bt_cam2world = toBTTransform(cam2world);
  cout << cam2world.matrix() << endl;

  vector<btVector3> corners_world = transform_btVectors(toBTs(corners_cam),bt_cam2world);
  vector<btVector3> ropePts = transform_btVectors(read_btVectors(first_rope),bt_cam2world);
  btVector3 halfExtents;
  btVector3 origin;

  verts2boxPars(corners_world,halfExtents,origin,.2*METERS);

  pcl::transformPointCloud(*cloud,*cloud,scaling(1*METERS)*cam2world);

  shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ropePts*METERS,.0075*METERS));
  vector<BulletObject::Ptr> children =  ropePtr->getChildren();



  shared_ptr<btDefaultMotionState> ms(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),origin*METERS)));
  shared_ptr<BulletObject> table(new BoxObject(0,halfExtents*METERS,ms));
  Scene s;

  initTrackingPlots();
  shared_ptr<btDefaultMotionState> ms0(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,10))));
  shared_ptr<btDefaultMotionState> ms1(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,10))));

  end0sphere.reset(new SphereObject(0,METERS*.015,ms0));
  end1sphere.reset(new SphereObject(0,METERS*.015,ms1));
  end0sphere->rigidBody->setCollisionFlags(btRigidBody::CF_NO_CONTACT_RESPONSE);
  end1sphere->rigidBody->setCollisionFlags(btRigidBody::CF_NO_CONTACT_RESPONSE);
  end0sphere->rigidBody->setActivationState(DISABLE_DEACTIVATION); //why?
  end1sphere->rigidBody->setActivationState(DISABLE_DEACTIVATION);

  //s.manip->state.debugDraw = false;
  PlotPoints::Ptr plot(new PlotPoints());
  plot->setPoints(cloud);

  s.env->add(plot);
  s.env->add(table);
  s.env->add(ropePtr);

  s.env->add(plots::forcelines);
  s.env->add(plots::targpts);
  s.env->add(end0sphere);
  s.env->add(end1sphere);
  s.startViewer();
  s.step(0);
  s.idle(true);

  int nSegs = ropePtr->bodies.size();
  btPoint2PointConstraint* ptp0 = new btPoint2PointConstraint(*ropePtr->bodies[0].get(),(ropePts[0]-ropePts[1])/2);
  btPoint2PointConstraint* ptp1 = new btPoint2PointConstraint(*ropePtr->bodies[nSegs-1].get(),(ropePts[nSegs-1]-ropePts[nSegs-2])/2);

  vector<btVector3> forces(ropePts.size());
  vector<btVector3> centers;
  vector<btVector3> oldcenters;
  EndInfo endinfo = {ptp0,ptp1,false,false};

  for (int t=0; ; t++) {
  cout << "flags: " << end0sphere->rigidBody->getFlags()  << endl;

    string pcdfile = pcd_listener.next();
    cout << "--------------- reading file " << pcdfile << "------------" << endl;
    cloud = readPCD(pcdfile);
    pcl::transformPointCloud(*cloud,*cloud,scaling(1*METERS)*cam2world);

    string ropefile = rope_listener.next();
    string endfile = ends_listener.next();

    ropePts =   transform_btVectors(read_btVectors(ropefile),bt_cam2world);
    ropePts = ropePts * METERS;

    updateEnds(endfile, s.env->bullet->dynamicsWorld, endinfo, bt_cam2world);

    plot->setPoints(cloud);
    ropePtr->getPts(centers);
    vector<bool> occs = checkOccluded(centers,cloud,cam2world);
    cout << "occlusions:";
    BOOST_FOREACH(bool occ, occs) cout << occ << " "; cout << endl;
    oldcenters = centers;
    for (int i=0; i < CFG2->nIter; i++) {

      vector<bool> occs = checkOccluded(centers,cloud,cam2world);
      vector<BulletObject::Ptr> children =  ropePtr->getChildren();
      for (int j=0; j<occs.size(); j++) {
	if (occs[j]) children[j]->setColor(0,0,0,1);
	else children[j]->setColor(1,1,1,1);
      }



      ropePtr->getPts(centers);
      calcOptImpulses(centers, ropePts, oldcenters, forces,occs);
      applyImpulses(ropePtr->bodies, forces, 1);
      s.step(.01,1,.01);



      


      cout << "iteration " << i  << endl;
      //     usleep(10*1000);
    }

    // s.manip->toggleIdle();
  }
}

