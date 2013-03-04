#include "conversions.h"
#include <boost/foreach.hpp>
#include "my_assert.h"
#include "my_exceptions.h"

Eigen::Matrix3f toEigenMatrix(const btMatrix3x3& basis)
{
	Eigen::Matrix3f rotation;
	btVector3 col0 = basis.getColumn(0);
	btVector3 col1 = basis.getColumn(1);
	btVector3 col2 = basis.getColumn(2);
	rotation.col(0) = toEigenVector(col0);
	rotation.col(1) = toEigenVector(col1);
	rotation.col(2) = toEigenVector(col2);
	return rotation;
}

btMatrix3x3 toBulletMatrix(const Eigen::Matrix3f& rotation)
{
	btMatrix3x3 basis;
	basis.setValue(rotation(0,0), rotation(0,1), rotation(0,2),
                   rotation(1,0), rotation(1,1), rotation(1,2),
                   rotation(2,0), rotation(2,1), rotation(2,2));
	return basis;
}

std::vector<btVector3> toBulletVectors(const std::vector< std::vector<float> >& in) {
  std::vector<btVector3> out(in.size());
  for (int i=0; i<in.size(); i++) out[i] = toBulletVector(in[i]);
  return out;
}

std::vector<btVector3> toBulletVectors(const std::vector< Eigen::Vector3f >& in) {
  std::vector<btVector3> out(in.size());
  for (int i=0; i<in.size(); i++) out[i] = toBulletVector(in[i]);
  return out;
}


std::vector<btVector3> toBulletVectors(const Eigen::MatrixXf& in) {
  std::vector<btVector3> out(in.rows());
  for (int row=0; row < in.rows(); row++)
    out[row] = btVector3(in(row,0), in(row,1), in(row,2));      
  return out;
}

std::vector< std::vector<float> > toVecVec(const std::vector<btVector3>& in) {
  std::vector< std::vector<float> > out(in.size());
  for (int i=0; i < in.size(); i++) {
    std::vector<float> row(3);
      for (int j=0; j<3; j++) {
	      row[j] = in[i].m_floats[j];
      }
    out[i] = row;
  }
  return out;
}

std::vector<float> toVec(const Eigen::VectorXf& in) {
  std::vector<float> out(in.rows());
  for (int i=0; i < out.size(); i++) out[i] = in[i];
  return out;
}

Eigen::VectorXf toVectorXf(const std::vector<float>& in) {
  Eigen::VectorXf out(in.size());
  for (int i=0; i<in.size(); i++) out[i] = in[i];
  return out;
}

btTransform toBulletTransform(const Eigen::Affine3f& affine) {
	btTransform t;
	t.setFromOpenGLMatrix(affine.data());
	return t;

//  Eigen::Vector3f transEig = affine.translation();
//  Eigen::Matrix3f rotEig = affine.rotation();
//  Eigen::Quaternionf quatEig = Eigen::Quaternionf(rotEig);
//  btVector3 transBullet = toBulletVector(transEig);
//  btQuaternion quatBullet = btQuaternion(quatEig.x(), quatEig.y(), quatEig.z(), quatEig.w());
//  return btTransform(quatBullet,transBullet);
}


Eigen::Affine3f toEigenTransform(const btTransform& transform) {
  Eigen::Affine3f t;
  transform.getOpenGLMatrix(t.data());
  return t;

//	btVector3 transBullet = transform.getOrigin();
//  btQuaternion quatBullet = transform.getRotation();
//  Eigen::Translation3f transEig;
//  transEig = Eigen::Translation3f(toEigenVector(transBullet));
//  Eigen::Matrix3f rotEig = Eigen::Quaternionf(quatBullet.w(),quatBullet.x(),quatBullet.y(),quatBullet.z()).toRotationMatrix();
//  Eigen::Affine3f out = transEig*rotEig;
//  return out;
}

std::vector<Eigen::Vector3f> toEigenVectors(const std::vector< std::vector<float> >& in) {
  std::vector<Eigen::Vector3f> out (in.size());
  for (int i=0; i < in.size(); i++) out[i] = toEigenVector(in[i]);
  return out;
}

std::vector<Eigen::Vector3f> toEigenVectors(const std::vector<btVector3>& in) {
  std::vector<Eigen::Vector3f> out(in.size());
  for (int i=0; i<in.size(); i++) out[i] = toEigenVector(in[i]);
  return out;
}

Eigen::MatrixXf toEigenMatrix(const std::vector<btVector3>& in) {
  Eigen::MatrixXf out(in.size(), 3);
  for (int i=0; i<in.size(); i++) out.row(i) = toEigenVector(in[i]);
  return out;
}

Eigen::MatrixXf toEigenMatrix(const std::vector<Eigen::Vector3f>& in) {
  Eigen::MatrixXf out(in.size(), 3);
  for (int i=0; i<in.size(); i++) out.row(i) = in[i].transpose();
  return out;
}

Eigen::MatrixXf toEigenMatrix(const std::vector< std::vector<float> >& in) {
  ENSURE(in.size() > 1) ;
  Eigen::MatrixXf out(in.size(),in[0].size()); 
  for (int i=0; i<in.size(); i++) 
    for (int j=0; j<in[0].size(); j++)
      out(i,j) = in[i][j];
  return out;
}
