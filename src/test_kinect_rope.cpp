#include "simplescene.h"
#include "file_reading.h"
#include "rope.h"
#include "unistd.h"
using boost::shared_ptr;
// make table and rope from kinect data

const string data_dir = "/home/joschu/Data/pink_rope";

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


  cout << "cam2world: " << endl;
  cout << cam2world[0][0] << " "  << cam2world[0][1] << " "  << cam2world[0][2] << endl;

  vector<btVector3> verts_world;
  for (int i=0; i<verts_cam.size(); i++) verts_world.push_back(cam2world*btVector3(verts_cam[i][0],verts_cam[i][1],verts_cam[i][2]));

  btAlignedObjectArray<btVector3> ctrlPts;
  for (int i=0; i<xyzs_cam.size(); i++) {
  btVector3 xyz_world = cam2world*btVector3(xyzs_cam[i][0],xyzs_cam[i][1],xyzs_cam[i][2]);
  cout << xyz_world[0] << " " << xyz_world[1] << " " << xyz_world[2] << endl;
ctrlPts.push_back(xyz_world);
  }
  btVector3 halfExtents;
  btVector3 origin;
  cout << "verts world:" << endl;
  cout << verts_world[0][0] << " "  << verts_world[0][1] << " "  << verts_world[0][2] << " " << endl;
  cout << verts_world[1][0] << " "  << verts_world[1][1] << " "  << verts_world[1][2] << " " << endl;
  cout << verts_world[2][0] << " "  << verts_world[2][1] << " "  << verts_world[2][2] << " " << endl;

  verts2boxPars(verts_world,halfExtents,origin,.2);
  origin[2] -= .1;

  shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ctrlPts,.01));


  shared_ptr<btMotionState> ms(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),origin)));
  shared_ptr<BulletObject> table(new BoxObject(0,halfExtents,ms));



  Scene s = Scene(false,false,false);
  s.env->add(ropePtr);
  s.env->add(table);
  s.step(0,100,.01);
  s.manip->toggleIdle();
  while (!s.viewer.done()) {

    s.step(.01,300,.001);
    usleep(10*1000);
  }

}
