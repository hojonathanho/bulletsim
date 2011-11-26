#include "simplescene.h"
#include "util.h"
#include "rope.h"
#include "unistd.h"
#include <Eigen/Dense>
#include "dist_math.h"
#include <math.h>
#include "plotting.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "pcl/common/eigen.h"
using boost::shared_ptr;
using namespace Eigen;
using namespace util;
// make table and rope from kinect data

const string data_dir = "../data";




void minRot(const btVector3& v1, const btVector3& v2, btMatrix3x3& m) {
  btScalar ang = v1.angle(v2);
  btVector3 ax = v2.cross(v1);
  m = btMatrix3x3(btQuaternion(ax,ang));

}

void verts2boxPars(const vector<btVector3>& verts, btVector3& halfExtents, btVector3& origin, btScalar thickness) {
  origin = (verts[0] + verts[2])/2;
  halfExtents = (verts[2] - verts[0]).absolute()/2;
  origin[2] -= thickness/2;
  halfExtents[0] *= 2;
  halfExtents[1] *= 2;
  halfExtents[2] = thickness/2;
}


MatrixXf toEigen(const vector<btVector3>& vecs) {
  MatrixXf out = MatrixXf(vecs.size(),3);
  for (int i=0; i < vecs.size(); i++) {
    //out.row(i).readArray(vecs[i].m_floats);
    out(i,0) = vecs[i].getX();
    out(i,1) = vecs[i].getY();
    out(i,2) = vecs[i].getZ();
  }
  return out;

}

void calcOptImpulses(const vector<btVector3>& ctrlPts, const vector<btVector3>& pointCloud, vector<btVector3>& forces) {
  MatrixXf ropePts = toEigen(ctrlPts);
  MatrixXf kinPts = toEigen(pointCloud);
  VectorXi indRope2Kin = argminAlongRows(pairwiseSquareDist(ropePts,kinPts));
  int nRope = ctrlPts.size();
  forces.clear();
  forces.resize(nRope);
  for (int i=0; i < nRope; i++) {
    forces[i] = pointCloud[indRope2Kin[i]] - ctrlPts[i];
  }
}

void applyImpulses(vector< shared_ptr<btRigidBody> > bodies, vector<btVector3> impulses, float multiplier) {
  for (int i=0; i<bodies.size(); i++) bodies[i]->applyCentralImpulse(impulses[i]*multiplier);
}

int main() {

  // load table plane coeffs
  // figure out the rotation matrix
  // load the rope
  // apply rotation to rope
  // make rope and add it to simulation


  vector< vector<float> > xyzs_cam;
  read_2d_array(xyzs_cam,data_dir+"/0003_xyz.txt"); //rope vertices
  vector<float> abcd;
  read_1d_array(abcd, data_dir + "/coeffs.txt"); // table coefficients
  vector< vector<float> > verts_cam;
  read_2d_array(verts_cam,data_dir+"/verts.txt"); // table vertices

  btMatrix3x3 cam2world;

  btVector3 normal(abcd[0],abcd[1],abcd[2]);
  minRot(btVector3(0,0,1),-normal,cam2world);

  cout << "cam2world:" << endl;
  print_matrix<btMatrix3x3,3> (cam2world);
  btVector3 trans(0,0,1.5);
  btTransform cam2world1(cam2world,trans);


  vector<btVector3> verts_world;
  for (int i=0; i<verts_cam.size(); i++) verts_world.push_back(cam2world1*btVector3(verts_cam[i][0],verts_cam[i][1],verts_cam[i][2]));

  vector<btVector3> ctrlPts;
  for (int i=0; i<xyzs_cam.size(); i+=2) {
  btVector3 xyz_world = cam2world1*btVector3(xyzs_cam[i][0],xyzs_cam[i][1],xyzs_cam[i][2]);
  //print_vector<btVector3,3>(xyz_world)
  ctrlPts.push_back(xyz_world);
  }
  btVector3 halfExtents;
  btVector3 origin;

  verts2boxPars(verts_world,halfExtents,origin,.2);





  const string pcdfile = "../data/0003.pcd";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdfile, *cloud) == -1) {
    PCL_ERROR(("couldn't read file " + pcdfile + "\n").c_str());
    return -1;
  }
  cout << "new cloud size: " << cloud->size();


  PlotPoints::Ptr plot(new PlotPoints());
  Affine3f T;
  Matrix3f M;
  for (int i=0; i<3; i++) for (int j=0; j<3; j++) M(i,j) = cam2world[i][j];
  Vector3f eig_trans(trans.getX(),trans.getY(),trans.getZ());


  DPRINT(M);
  btVector3 vbullet = btVector3(1,2,3);
  Vector3f veigen = Vector3f(1,2,3);
  DPRINT(M*veigen);
  cout << "cam2world*vbullet: " << (M*veigen) << endl;


  T = Translation3f(eig_trans)*M;
  //cout << M << endl;
  cout << "before" << endl;
  pcl::PointXYZRGB pt = cloud->at(10);
  cout << pt.x QQ pt.y QQ pt.z QQ (int)pt.r QQ (int)pt.g QQ (int)pt.b << endl;

  vbullet = btVector3(pt.x,pt.y,pt.z);

  pcl::transformPointCloud(*cloud,*cloud,T);
  cout << "after" << endl;
  pt = cloud->at(10);
  cout << pt.x QQ pt.y QQ pt.z QQ (int)pt.r QQ (int)pt.g QQ (int)pt.b << endl;

  cout << "transformed with bullet " << endl << (cam2world*vbullet) << endl;

  plot->setPoints(cloud);




  shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ctrlPts,.01));
  shared_ptr<btMotionState> ms(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),origin)));

  shared_ptr<BulletObject> table(new BoxObject(0,halfExtents,ms));
  Scene s = Scene(false,false,false);
  s.manip->state.debugDraw = false;

  s.env->add(plot);
  s.env->add(table);
  s.env->add(ropePtr);
  s.viewerLoop();

  vector<btVector3> pointCloud;
  vector<btVector3> forces(ctrlPts.size());
  // point cloud is a little offset from rope
  vector<btVector3> centers;



  s.step(0,0,.01);
  vector<btRigidBody> bodies;
  for (int t=0; t < 100 && !s.viewer.done(); t++) {

    pointCloud.clear();
    for (int i=0; i < ctrlPts.size(); i++) pointCloud.push_back(ctrlPts[i]+btVector3(0*sin(t/3.),0,0));


    ropePtr->getPts(centers);
    calcOptImpulses(centers, pointCloud, forces);
    DPRINT(forces[0]);
    DPRINT(forces[1]);
    applyImpulses(ropePtr->bodies, forces, 1);
    s.step(.01,1,.01);
    cout << t << endl;
  }
  /*
  s.manip->toggleIdle();
  while (!s.viewer.done()) {
    s.step(.01,300,.001);
    usleep(10*1000);
  }
  */

}
