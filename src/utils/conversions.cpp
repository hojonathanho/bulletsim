#include "conversions.h"
#include <boost/foreach.hpp>
#include "utils/my_assert.h"
#include "utils/my_exceptions.h"

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

osg::Matrix toOsgMatrix( const btTransform& t )
{
    btScalar ogl[ 16 ];
    t.getOpenGLMatrix( ogl );
    osg::Matrix m( ogl );
    return m;
}

btTransform toBtTransform( const osg::Matrix& m )
{
    const osg::Matrix::value_type* oPtr = m.ptr();
    btScalar bPtr[ 16 ];
    int idx;
    for (idx=0; idx<16; idx++)
        bPtr[ idx ] = oPtr[ idx ];
    btTransform t;
    t.setFromOpenGLMatrix( bPtr );
    return t;
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

btTransform toBulletTransform(const geometry_msgs::Transform& in) {
	btTransform out;
	out.setOrigin(btVector3(in.translation.x, in.translation.y, in.translation.z));
	out.setRotation(btQuaternion(in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));
	return out;
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

geometry_msgs::Transform toROSTransform(const Eigen::Affine3f& in) {
  Eigen::Vector3f trans = in.translation();
  Eigen::Matrix3f rot = in.rotation();
  Eigen::Quaternionf quat = Eigen::Quaternionf(rot);
  geometry_msgs::Transform out;
  out.translation.x = trans(0);
  out.translation.y = trans(1);
  out.translation.z = trans(3);
  out.rotation.x = quat.x();
  out.rotation.y = quat.y();
  out.rotation.z = quat.z();
  out.rotation.w = quat.w();
  return out;
}

geometry_msgs::Transform toROSTransform(const btTransform& in) {
  btVector3 trans = in.getOrigin();
  btQuaternion quat = in.getRotation();
  geometry_msgs::Transform out;
  out.translation.x = trans.x();
  out.translation.y = trans.y();
  out.translation.z = trans.z();
  out.rotation.x = quat.x();
  out.rotation.y = quat.y();
  out.rotation.z = quat.z();
  out.rotation.w = quat.w();
  return out;
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


std::vector<btVector3> toBulletVectors(const std::vector<geometry_msgs::Point>& in) {
	std::vector<btVector3> out(in.size());
	for (int i=0; i < in.size(); ++i) out[i] = btVector3(in[i].x, in[i].y, in[i].z);
	return out;
}
std::vector<btVector3> toBulletVectors(const std::vector<geometry_msgs::Point32>& in) {
	std::vector<btVector3> out(in.size());
	for (int i=0; i < in.size(); ++i) out[i] = btVector3(in[i].x, in[i].y, in[i].z);
	return out;
}

std::vector<geometry_msgs::Point> toROSPoints(const std::vector<btVector3>& in) {
	std::vector<geometry_msgs::Point> out(in.size());
	for (int i=0; i < in.size(); ++i) {
		out[i].x = in[i].x();
		out[i].y = in[i].y();
		out[i].z = in[i].z();
	}
	return out;
}

std::vector<geometry_msgs::Point32> toROSPoints32(const std::vector<btVector3>& in) {
	std::vector<geometry_msgs::Point32> out(in.size());
	for (int i=0; i < in.size(); ++i) {
		out[i].x = in[i].x();
		out[i].y = in[i].y();
		out[i].z = in[i].z();
	}
	return out;
}

geometry_msgs::Point32 toROSPoint32(const btVector3& vec) {
	geometry_msgs::Point32 g_pt;
	g_pt.x = vec.x();
	g_pt.y = vec.y();
	g_pt.z = vec.z();
	return g_pt;
}

geometry_msgs::Point toROSPoint(const btVector3& vec) {
	geometry_msgs::Point g_pt;
	g_pt.x = vec.x();
	g_pt.y = vec.y();
	g_pt.z = vec.z();
	return g_pt;
}
