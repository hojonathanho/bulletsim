#include "get_table.h"
#include "geom.h"
#include "utils1.h"
#include "comm.h"

#include "simplescene.h"
#include "util.h"
#include "rope.h"
#include "unistd.h"
#include "dist_math.h"
#include "plotting.h"


#include <math.h>
#include <pcl/visualization/cloud_viewer.h>
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

PlotLines::Ptr forcelines;
PlotPoints::Ptr targpts;

const float METERS = 10;

template<class T, class S>
vector<T> operator*(vector<T>& xs, S p) {
  vector<T> ys;
  ys.resize(xs.size());
  for (int i=0; i<xs.size(); i++) ys[i] = xs[i]*p;
  return ys;
}

void verts2boxPars(const vector<btVector3>& verts, btVector3& halfExtents, btVector3& origin, btScalar thickness) {
  origin = (verts[0] + verts[2])/2;
  halfExtents = (verts[2] - verts[0]).absolute()/2;
  origin[2] -= thickness/2;
  halfExtents[2] = thickness/2;
}

Affine3f getCam2World(const vector<Vector3f>& tableVerts, Vector3f& normal){
  Matrix3f world2cam_rot;
  Vector3f newY = tableVerts[1] - tableVerts[0];
  Vector3f newX = tableVerts[3] - tableVerts[0];
  Vector3f newZ = newX.cross(newY);
  newX.normalize(); newY.normalize(); newZ.normalize();
  world2cam_rot.col(0) = newX;
  world2cam_rot.col(1) = newY;
  world2cam_rot.col(2) = newZ;
  cout << "det: " << world2cam_rot.determinant() << endl;
  cout << "dot: " << newX.dot(newY) << endl;
  Matrix3f cam2world_rot = world2cam_rot.inverse();

  float tz = 1-(cam2world_rot*tableVerts[0])[2];
  return Translation3f(0,0,tz)*cam2world_rot;
}

Affine3f scaling(float s) {
  Affine3f T;
  T = s*Matrix3f::Identity();
  return T;
}

void calcOptImpulses(const vector<btVector3>& ctrlPts, const vector<btVector3>& pointCloud, vector<btVector3>& forces) {
  MatrixXf ropePts = toEigens(ctrlPts);
  MatrixXf kinPts = toEigens(pointCloud);
  VectorXi indRope2Kin = argminAlongRows(pairwiseSquareDist(ropePts,kinPts));

  vector<btVector3> plot_srcs;
  vector<btVector3> plot_targs;

  int nRope = ctrlPts.size();
  forces.resize(nRope);
  for (int i_rope=0; i_rope < nRope; i_rope++) {
    btVector3 cloud_pt = pointCloud[indRope2Kin[i_rope]];
    btVector3 rope_pt = ctrlPts[i_rope];
    forces[i_rope] = cloud_pt - rope_pt;
    plot_srcs.push_back(rope_pt);
    plot_targs.push_back(cloud_pt);
  }

  VectorXi indKin2Rope = argminAlongRows(pairwiseSquareDist(kinPts,ropePts));
  for (int i_cloud=0; i_cloud < indKin2Rope.size(); i_cloud++) {
    int i_rope = indKin2Rope[i_cloud];
    btVector3 rope_pt = ctrlPts[i_rope];
    btVector3 cloud_pt = pointCloud[i_cloud];
    forces[i_rope] += cloud_pt - rope_pt;
    plot_srcs.push_back(rope_pt);
    plot_targs.push_back(cloud_pt);
  }

  forcelines->setPoints(plot_srcs,plot_targs);
  targpts->setPoints(pointCloud);

}

void applyImpulses(vector< shared_ptr<btRigidBody> > bodies, vector<btVector3> impulses, float multiplier) {
  for (int i=0; i<bodies.size(); i++) bodies[i]->applyCentralImpulse(impulses[i]*multiplier);
}

int main() {


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
  // btTransform cam2world_bullet;
  // cam2world_bullet.setIdentity();

  vector<btVector3> corners_world = transform_btVectors(toBTs(corners_cam),bt_cam2world);

  BOOST_FOREACH(btVector3 vec, corners_world) cout << vec.getX() QQ vec.getY() QQ vec.getZ() << endl;
  // vector< btVector3 > xyzs_cam;
  // read_btVectors(xyzs_cam,initial_ropefile); //rope vertices

  vector<btVector3> ropePts = transform_btVectors(read_btVectors(first_rope),bt_cam2world);
  vector<btVector3> endPts = transform_btVectors(read_btVectors(first_ends),bt_cam2world);
  // btVector3 normal(abcd[0],abcd[1],abcd[2]);

  BOOST_FOREACH(btVector3 vec, ropePts) cout << vec.getX() QQ vec.getY() QQ vec.getZ() << endl;

  // cout << "cam2world:" << endl;
  // print_matrix<btMatrix3x3,3> (cam2world);
  // btVector3 trans(0,0,1.5);
  // btTransform cam2world1(cam2world,trans);


  // vector<btVector3> verts_world;
  // for (int i=0; i<verts_cam.size(); i++) verts_world.push_back(cam2world1*btVector3(verts_cam[i][0],verts_cam[i][1],verts_cam[i][2]));

  // vector<btVector3> ctrlPts;
  // for (int i=0; i<xyzs_cam.size(); i+=2) {
  //   btVector3 xyz_world = cam2world1*btVector3(xyzs_cam[i][0],xyzs_cam[i][1],xyzs_cam[i][2]);
  //   ctrlPts.push_back(xyz_world);
  // }
  btVector3 halfExtents;
  btVector3 origin;

  verts2boxPars(corners_world,halfExtents,origin,.2);

  pcl::transformPointCloud(*cloud,*cloud,scaling(1*METERS)*cam2world);

  //   pcl::transformPointCloud(*cloud,*cloud,cam2world);

  // Affine3f T;
  // Matrix3f M;
  // for (int i=0; i<3; i++) for (int j=0; j<3; j++) M(i,j) = cam2world[i][j];
  // Vector3f eig_trans(trans.getX(),trans.getY(),trans.getZ());
  // T = Translation3f(eig_trans)*M;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr rectCloud(new pcl::PointCloud<pcl::PointXYZ>);
  // BOOST_FOREACH(btVector3 w, corners_world) rectCloud->push_back(pcl::PointXYZ(w[0],w[1],w[2]));
  // pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
  // viewer.addPointCloud (cloud);
  // viewer.addPolygon<pcl::PointXYZ>(rectCloud,255,0,0);
  // viewer.spin();



  // // const string pcdfile = "../data/0003.pcd";
  // // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  // // if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdfile, *cloud) == -1) {
  // //   PCL_ERROR(("couldn't read file " + pcdfile + "\n").c_str());
  // //   return -1;
  // // }
  // // cout << "new cloud size: " << cloud->size();


  // /////////////// put stuf into scene //////////////////

  shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ropePts*METERS,.0075*METERS));
  shared_ptr<btMotionState> ms(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),origin*METERS)));
  shared_ptr<BulletObject> table(new BoxObject(0,halfExtents*METERS,ms));
  Scene s(false,false,false,1);
  s.manip->state.debugDraw = false;
  PlotPoints::Ptr plot(new PlotPoints());
  plot->setPoints(cloud);

  s.env->add(plot);
  s.env->add(table);
  s.env->add(ropePtr);
  forcelines.reset(new PlotLines(3));
  targpts.reset(new PlotPoints(20));
  s.env->add(forcelines);
  s.env->add(targpts);

  s.step(0,1,.01);
  s.manip->toggleIdle();

  //Declarations for stuff in loops

  btPoint2PointConstraint* ptp1 = new btPoint2PointConstraint(*ropePtr->bodies[0].get(),btVector3(0,0,0));
  btPoint2PointConstraint* ptp2 = new btPoint2PointConstraint(*ropePtr->bodies[ropePtr->bodies.size()-1].get(),btVector3(0,0,0));
  s.env->bullet->dynamicsWorld->addConstraint(ptp1);
  s.env->bullet->dynamicsWorld->addConstraint(ptp2);

  vector<btVector3> forces(ropePts.size());
  vector<btVector3> centers;

  for (int t=0; ; t++) {
    // for (int t=0;  t<pcdfiles.size(); t++) {
    string pcdfile = pcd_listener.next();
    cout << "--------------- reading file " << pcdfile << "------------" << endl;
    cloud = readPCD(pcdfile);
    pcl::transformPointCloud(*cloud,*cloud,scaling(1*METERS)*cam2world);

    string ropefile = rope_listener.next();
    string endfile = ends_listener.next();

    ropePts =   transform_btVectors(read_btVectors(ropefile),bt_cam2world);
    ropePts = ropePts * METERS;
    cout<<"pts: " << endl;
    BOOST_FOREACH(btVector3 pt, ropePts) cout << pt.getX() << " " << pt.getY() << " " << pt.getZ() << endl;
    endPts =   transform_btVectors(read_btVectors(endfile),bt_cam2world);
    endPts = endPts * METERS;
    ptp1->setPivotB(endPts[0]);
    ptp2->setPivotB(endPts[1]);

    plot->setPoints(cloud);

    for (int i=0; i < 20; i++) {




      ropePtr->getPts(centers);
      calcOptImpulses(centers, ropePts, forces);

      applyImpulses(ropePtr->bodies, forces, 5);

      s.step(.01,1,.01);



      


      cout << "iteration " << i  << endl;
      //     usleep(10*1000);
    }

    s.manip->toggleIdle();
  }
}

