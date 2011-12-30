#include "userconfig.h"
#include "simplescene.h"
#include "util.h"
#include "plotting.h"
#include "tracking.h"
#include "softbodies.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBody.h>

#include "get_table.h"
#include "clouds/geom.h"
#include "utils_perception.h"
#include "comm.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <boost/foreach.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/lambda/lambda.hpp>
#include <algorithm>
// #include <boost/lambda/bind.hpp>

using boost::shared_ptr;
using namespace Eigen;
using namespace util;
// make table and rope from kinect data

float METERS;

vector<btVector3> getNodes(BulletSoftObject::Ptr cloth) {
  btAlignedObjectArray<btSoftBody::Node> m_nodes = cloth->softBody->m_nodes;
  vector<btVector3> out(m_nodes.size());
  for (int i=0; i < m_nodes.size(); i++) out[i] = m_nodes[i].m_x;
  return out;
}

void applySoftBodyImpulses(BulletSoftObject::Ptr cloth, vector<btVector3> impulses, float multiplier) {
  for (int i=0; i<impulses.size(); i++)
    cloth->softBody->addForce(impulses[i]*multiplier,i);
}

void dampNodeVelocities(BulletSoftObject::Ptr cloth, vector<btVector3> impulses, float multiplier) {
  for (int i=0; i<impulses.size(); i++)
    cloth->softBody->m_nodes[i].m_v *= multiplier;
}


int main(int argc, char *argv[]) {
  CFG2->read(argc,argv);
  CFG2->scene.enableIK = CFG2->scene.enableHaptics = CFG2->scene.enableRobot = false;
  CFG2->scene.scale = 10;
  CFG2->bullet.gravity = btVector3(0,0,0);
  setConfigData(CFG2);
  METERS = CFG2->scene.scale;

  comm::Listener pcd_listener("pcds");
  comm::Listener towelpts_listener("towelpts");

    cout << "scale: " << METERS << endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = readPCD(pcd_listener.next(MODE_NOUP));

  vector<Vector3f> corners_cam;
  Vector3f normal;
  //  getTable(cloud,corners_cam,normal);
  normal = Vector3f( -0.00976028, 0.593937, 0.804452);
  corners_cam.push_back(Vector3f(-0.354021, 0.289245, 0.512503));
  corners_cam.push_back(Vector3f(-0.438022,-0.44194,1.05133));
  corners_cam.push_back(Vector3f(0.210096,-0.48641,1.09202));
  corners_cam.push_back(Vector3f(0.294098,0.244775,0.553199));

  // cout << "normal" << endl << normal << endl;
  // cout << "corners_cam" << endl ;
  //   BOOST_FOREACH(Vector3f vec, corners_cam) cout << vec;
  // cout << endl;

  Affine3f cam2world = getCam2World(corners_cam,normal,METERS);
  btTransform bt_cam2world = toBTTransform(cam2world);
  cout << cam2world.matrix() << endl;

  vector<btVector3> corners_world = transform_btVectors(toBTs(corners_cam),bt_cam2world);
  btVector3 halfExtents;
  btVector3 origin;

  BOOST_FOREACH(Vector3f vec, corners_cam) cout << vec[0] QQ vec[1] QQ vec[2] << endl;
  vector<btVector3> towelCorners_cam;
  vector<btVector3> towelCorners_world;
  read_btVectors(towelCorners_cam,"/home/joschu/Data/towel/corners.txt");
  towelCorners_world = transform_btVectors(towelCorners_cam, bt_cam2world);
  //towelCorners_world = transform_btVectors(towelPts_world,bt_cam2world);
  
  verts2boxPars(corners_world,halfExtents,origin,.2*METERS);
  pcl::transformPointCloud(*cloud,*cloud,cam2world);

  shared_ptr<btDefaultMotionState> ms(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),origin)));  shared_ptr<BulletObject> table(new BoxObject(0,halfExtents,ms));
  Scene scene;
  btVector3 offset(0,.1*METERS,.01*METERS);
#define TWC towelCorners_world
  btSoftBody* psb=btSoftBodyHelpers::CreatePatch(scene.env->bullet->softBodyWorldInfo,
						 TWC[0]+offset,
						 TWC[1]+offset,
						 TWC[2]+offset,
						 TWC[3]+offset,
						 17, 17,
						 0/*1+2+4+8*/, true);
#undef TWC
  psb->getCollisionShape()->setMargin(.01*METERS);
  btSoftBody::Material* pm=psb->appendMaterial();
  pm->m_kLST		=	0.1;
  pm->m_kAST = 0.1;
  //	pm->m_flags		-=	btSoftBody::fMaterial::DebugDraw;
  psb->generateBendingConstraints(2, pm);
  psb->setTotalMass(1);



  BulletSoftObject::Ptr towel = BulletSoftObject::Ptr(new BulletSoftObject(psb));


  initTrackingPlots();
  shared_ptr<btDefaultMotionState> ms0(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,10))));
  shared_ptr<btDefaultMotionState> ms1(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,10))));

  PlotPoints::Ptr plot(new PlotPoints());
  plot->setPoints(cloud);

  scene.env->add(plot);
  scene.env->add(table);
  scene.env->add(towel);

  scene.env->add(plots::forcelines);
  scene.env->add(plots::targpts);

  scene.startViewer();
  scene.step(0);

  int nNodes = towel->softBody->m_nodes.size();

  vector<btVector3> forces(nNodes);
  vector<btVector3> centers(nNodes);
  vector<btVector3> oldcenters(nNodes);
  vector<bool> occs(nNodes,false);

  for (int t=0; ; t++){
    scene.idle(true);

    cloud = readPCD(pcd_listener.next());
    pcl::transformPointCloud(*cloud,*cloud,cam2world);
    plot->setPoints(cloud);

    vector<btVector3> targPts = transform_btVectors(read_btVectors(towelpts_listener.next()),bt_cam2world);
    DPRINT(targPts.size());


    oldcenters = centers;
    for (int i=0; i < CFG2->nIter; i++) {
      centers = getNodes(towel);

      forces.clear();
      calcOptImpulses(centers,targPts,oldcenters, forces,occs);

      applySoftBodyImpulses(towel, forces, CFG2->mult);
      dampNodeVelocities(towel, forces, .5);

      scene.step(.01,1,.01);
      cout << "iteration " << i  << endl;

    }

  }
}

