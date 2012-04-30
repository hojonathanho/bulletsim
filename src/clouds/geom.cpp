#include "geom.h"
#include <boost/foreach.hpp>
#include <iostream>
#include <math.h>
#include <Wm5Core.h>
#include <Wm5Mathematics.h>
#include <Wm5ContMinBox2.h>

using namespace Wm5;
using namespace std;


void perpBasis(const Vector3f& v1, Vector3f& v2, Vector3f& v3) {
  v2 = Vector3f(-v1[1],v1[0],0);
  if (v2.Length() == 0) v2 = Vector3f(1,0,0);
  else v2.Normalize();
  v3 = v1.Cross(v2);
}


void minEncRect(const vector<Vector3f>& pts3d, const Vector4f& abcd, vector<Vector3f>& verts3d) {
  // ax+by+cz + d = 0
  // project points to 2d
  Vector3f abc(abcd[0],abcd[1],abcd[2]);
  Vector3f normal = abc/abc.Length();
  float q = abcd[3]/abc.Length();
  //normal . (x,y,z) + q = 0
  
  Vector3f va, vb;
  perpBasis(normal,va,vb);
  int nPts = pts3d.size();
  Vector2f* pts2d = new Vector2f[nPts];
  for (int i=0; i<nPts; i++) {
    pts2d[i] = Vector2f(pts3d[i].Dot(va),pts3d[i].Dot(vb));
    //    cout << i<< " " << pts2d[i] << endl;
  }

  Box2<float> minBox = MinBox2<float>(nPts,pts2d,.005,Query::QT_REAL,false);

  Vector2f* verts2d = new Vector2f[4];
  minBox.ComputeVertices(verts2d);

  verts3d.clear();
  for (int i=0; i<4; i++) {
    verts3d.push_back(va*verts2d[i][0] + vb*verts2d[i][1] - normal*q);
  }

  delete[] verts2d;
  delete[] pts2d;
}

void minEncRect(const std::vector<Eigen::Vector3f>& pts3d, const Eigen::Vector4f& abcd, std::vector<Eigen::Vector3f>& verts3d) {

  std::vector<Wm5::Vector3f> Wm_pts3d; 
  Wm5::Vector4f Wm_abcd(abcd[0],abcd[1],abcd[2],abcd[3]); 
  BOOST_FOREACH(Eigen::Vector3f pt, pts3d) Wm_pts3d.push_back(Wm5::Vector3f(pt[0],pt[1],pt[2]));
  std::vector<Wm5::Vector3f> Wm_verts3d;

  minEncRect(Wm_pts3d,Wm_abcd,Wm_verts3d);
  BOOST_FOREACH(Wm5::Vector3f vert, Wm_verts3d) verts3d.push_back(Eigen::Vector3f(vert[0],vert[1],vert[2]));

}


float angBetween(const Vector3f& v1, const Vector3f& v2) {
  return Mathf::ACos(v1.Dot(v2));
}

float angBetween(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
  return acos(v1.dot(v2));
}

// fails if v1==v2 or v1==-v2
void minRot(const Vector3f& v1, const Vector3f& v2, Matrix3f m) {
  float ang = angBetween(v1,v2);
  Vector3f ax = v2.Cross(v1);
  m = Matrix3f(ax,ang);
}


void minRot(const btVector3& v1, const btVector3& v2, btMatrix3x3& m) {
  btScalar ang = v1.angle(v2);
  btVector3 ax = v2.cross(v1);
  m = btMatrix3x3(btQuaternion(ax,ang));
}

void minRot(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, Eigen::Matrix3f& m) {
  float ang = angBetween(v1,v2);
  Eigen::Vector3f ax = v2.cross(v1);
  m = Eigen::AngleAxisf(ang,ax).toRotationMatrix();
}

std::vector<Eigen::Vector3f> getCorners(const std::vector<Eigen::Vector3f>& pts) {
  std::vector<Eigen::Vector3f> out;
  const Eigen::Vector4f abcd(0,0,1,-pts[0][2]);
  cout << "warning: assuming all points have same z" << endl;
  minEncRect(pts, abcd, out);
  return out;
}



/*
int main() {
  cout << "hi" << endl;
  Vector3f v1(1,0,0);
  Vector3f v2;
  Vector3f v3;
  perpBasis(v1,v2,v3);
  cout << v1 << endl;
  cout << v2 << endl;
  cout << v3 << endl;
  int n;
  cin >> n;
  Vector2f* arr = new Vector2f[n];
  arr[0] = Vector2f(2,3);
}
*/
