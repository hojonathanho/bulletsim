#include "simplescene.h"
#include "vector_io.h"
#include "rope.h"
#include "unistd.h"
#include <Eigen/Dense>
#include "dist_math.h"
#include <math.h>
using boost::shared_ptr;
using namespace Eigen;
// make table and rope from kinect data

const string data_dir = "/home/joschu/Data/pink_rope";

ostream &operator<<( ostream &out, btVector3 &v ) {
  out << "[" << v.getX() << " " << v.getY() << " " << v.getZ() << "]";
}


void minRot(const btVector3& v1, const btVector3& v2, btMatrix3x3& m) {
  btScalar ang = v1.angle(v2);
  btVector3 ax = v2.cross(v1);
  m = btMatrix3x3(btQuaternion(ax,ang));

}

void verts2boxPars(const vector<btVector3>& verts, btVector3& halfExtents, btVector3& origin, btScalar thickness) {
  origin = (verts[0] + verts[2])/2;
  halfExtents = (verts[2] - verts[0]).absolute()/2;
  origin[2] -= thickness/2;
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

void applyImpulses(vector< shared_ptr<btRigidBody> > bodies, vector<btVector3> impulses) {
  for (int i=0; i<bodies.size(); i++) bodies[i]->applyCentralImpulse(impulses[i]);
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
  minRot(btVector3(0,0,1),normal,cam2world);

  cout << "cam2world:" << endl;
  print_matrix<btMatrix3x3,3> (cam2world);

  vector<btVector3> verts_world;
  for (int i=0; i<verts_cam.size(); i++) verts_world.push_back(cam2world*btVector3(verts_cam[i][0],verts_cam[i][1],verts_cam[i][2]));

  vector<btVector3> ctrlPts;
  for (int i=0; i<xyzs_cam.size(); i++) {
  btVector3 xyz_world = cam2world*btVector3(xyzs_cam[i][0],xyzs_cam[i][1],xyzs_cam[i][2]);
  //print_vector<btVector3,3>(xyz_world)
    ctrlPts.push_back(xyz_world);
  }
  btVector3 halfExtents;
  btVector3 origin;

  verts2boxPars(verts_world,halfExtents,origin,.2);
  origin[2] -= .01;

  shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ctrlPts,.01));
  shared_ptr<btMotionState> ms(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),origin)));
  shared_ptr<BulletObject> table(new BoxObject(0,halfExtents,ms));
  Scene s = Scene(false,false,false);
  s.env->add(ropePtr);
  s.env->add(table);
  s.step(0,100,.01);

  vector<btVector3> pointCloud;
  vector<btVector3> forces(ctrlPts.size());
  // point cloud is a little offset from rope
  vector<btVector3> centers;

  s.step(0,0,.01);
  vector<btRigidBody> bodies;
  for (int t=0; t < 100; t++) {

    pointCloud.clear();
    for (int i=0; i < ctrlPts.size(); i++) pointCloud.push_back(ctrlPts[i]+btVector3(.2*sin(t/3.),0,0));


    ropePtr->getPts(centers);
    calcOptImpulses(centers, pointCloud, forces);
    DPRINT(forces[0]);
    DPRINT(forces[1]);
    applyImpulses(ropePtr->bodies, forces);
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
