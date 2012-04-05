#pragma once
#include <boost/foreach.hpp>
#include <btBulletDynamicsCommon.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <osg/Geometry>
#include "utils/my_assert.h"
#include "utils/my_exceptions.h"

inline btVector3 toBulletVector(const std::vector<float>& vec) {return btVector3(vec[0],vec[1],vec[2]);}
inline btVector3 toBulletVector(const Eigen::Vector3f& vec) {return btVector3(vec[0],vec[1],vec[2]);}
inline btVector3 toBulletVector(const osg::Vec3d &v) { return btVector3(v.x(), v.y(), v.z()); }

inline Eigen::Vector3f toEigenVector(const std::vector<float>& vec) {return Eigen::Vector3f(vec[0],vec[1],vec[2]);}
inline Eigen::Vector3f toEigenVector(const btVector3& vec) {return Eigen::Vector3f(vec.x(),vec.y(),vec.z());}

inline osg::Vec3f toOSGVector(const btVector3 &v) { return osg::Vec3f(v.x(), v.y(), v.z()); }

inline osg::Matrix toOsgMatrix( const btTransform& t )
{
    btScalar ogl[ 16 ];
    t.getOpenGLMatrix( ogl );
    osg::Matrix m( ogl );
    return m;
}

inline btTransform toBtTransform( const osg::Matrix& m )
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



inline std::vector<btVector3> toBulletVectors(const std::vector< std::vector<float> >& in) {
  std::vector<btVector3> out(in.size());
  for (int i=0; i<in.size(); i++) out[i] = toBulletVector(in[i]);
  return out;
}

inline std::vector<btVector3> toBulletVectors(const std::vector< Eigen::Vector3f >& in) {
  std::vector<btVector3> out(in.size());
  for (int i=0; i<in.size(); i++) out[i] = toBulletVector(in[i]);
  return out;
}


inline std::vector<btVector3> toBulletVectors(const Eigen::MatrixXf& in) {
  std::vector<btVector3> out(in.rows());
  for (int row=0; row < in.rows(); row++)
    out[row] = btVector3(in(row,0), in(row,1), in(row,2));      
  return out;
}

inline std::vector< std::vector<float> > toVecVec(const std::vector<btVector3>& in) {
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

inline std::vector<float> toVec(const Eigen::VectorXf& in) {
  std::vector<float> out(in.rows());
  for (int i=0; i < out.size(); i++) out[i] = in[i];
  return out;
}

inline Eigen::VectorXf toVectorXf(const std::vector<float>& in) {
  Eigen::VectorXf out(in.size());
  for (int i=0; i<in.size(); i++) out[i] = in[i];
  return out;
}

inline btTransform toBulletTransform(const Eigen::Affine3f& affine) {
  Eigen::Vector3f transEig = affine.translation();
  Eigen::Matrix3f rotEig = affine.rotation();
  Eigen::Quaternionf quatEig = Eigen::Quaternionf(rotEig);
  btVector3 transBullet = toBulletVector(transEig);
  btQuaternion quatBullet = btQuaternion(quatEig.x(), quatEig.y(), quatEig.z(), quatEig.w());
  return btTransform(quatBullet,transBullet);
}

inline Eigen::Affine3f toEigenTransform(const btTransform& transform) {
  btVector3 transBullet = transform.getOrigin();
  btQuaternion quatBullet = transform.getRotation();
  Eigen::Translation3f transEig;
  transEig = Eigen::Translation3f(toEigenVector(transBullet));
  Eigen::Matrix3f rotEig = Eigen::Quaternionf(quatBullet.w(),quatBullet.x(),quatBullet.y(),quatBullet.z()).toRotationMatrix();
  Eigen::Affine3f out = transEig*rotEig;
  return out;
}


inline std::vector<Eigen::Vector3f> toEigenVectors(const std::vector< std::vector<float> >& in) {
  std::vector<Eigen::Vector3f> out (in.size());
  for (int i=0; i < in.size(); i++) out[i] = toEigenVector(in[i]);
  return out;
}

inline std::vector<Eigen::Vector3f> toEigenVectors(const std::vector<btVector3>& in) {
  std::vector<Eigen::Vector3f> out(in.size());
  for (int i=0; i<in.size(); i++) out[i] = toEigenVector(in[i]);
  return out;
}

inline Eigen::MatrixXf toEigenMatrix(const std::vector<btVector3>& in) {
  Eigen::MatrixXf out(in.size(), 3);
  for (int i=0; i<in.size(); i++) out.row(i) = toEigenVector(in[i]);
  return out;
}

inline Eigen::MatrixXf toEigenMatrix(const std::vector< std::vector<float> >& in) {
  ENSURE(in.size() > 1) ;
  Eigen::MatrixXf out(in.size(),in[0].size()); 
  for (int i=0; i<in.size(); i++) 
    for (int j=0; j<in[0].size(); j++)
      out(i,j) = in[i][j];
  return out;
}
