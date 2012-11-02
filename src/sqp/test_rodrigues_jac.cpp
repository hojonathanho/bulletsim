#include <btBulletCollisionCommon.h>
#include "utils_sqp.h"
#include <Eigen/Geometry>
#include "functions.h"
#include "sqp_fwd.h"
using namespace std;
using namespace Eigen;

Vector3d calcPtWorld(const Vector3d& ptLocal, const Vector3d& centerWorld, const Vector3d& rod) {
  return Eigen::Translation3d(centerWorld) * AngleAxisd(rod.norm(), rod.normalized()) * ptLocal;
}




Matrix3d rod2mat(const Vector3d& rod) {
  return AngleAxisd(rod.norm(), rod.normalized()).matrix();
}


Matrix3d rotnumjac(const Vector3d& ptLocal, const Vector3d& rod) {
  struct F : public fVectorOfVector {
    VectorXd m_ptLocal;
    F(const VectorXd& ptLocal1) : m_ptLocal(ptLocal1) {}
    VectorXd operator()(const VectorXd& rod1) const {
      return calcPtWorld(m_ptLocal, Vector3d(0,0,0), rod1);
    }
  };
  return calcJacobian(F(ptLocal), rod);
}

Matrix3d rotjacworld(const Vector3d& ptWorld, const Vector3d& centerWorld, const Vector3d& rod) {
  Affine3d A;
  A = Eigen::Translation3d(centerWorld) * AngleAxisd(rod.norm(), rod.normalized());
  Vector3d ptLocal = A.inverse() * ptWorld;
  return rotnumjac(ptLocal, rod);
}



int main() {
  Vector3d ptLocal(1,1,1);
  Vector3d centerWorld(10,0,0);
  Vector3d rod(SIMD_PI, 0, 0);
  Vector3d ptWorld = calcPtWorld(ptLocal, centerWorld, rod);
  cout << ptWorld.transpose() << endl;
  cout << rotnumjac(ptLocal, rod).transpose() << endl;
  cout << rotjacworld(ptWorld, centerWorld, rod).transpose() << endl;
}
