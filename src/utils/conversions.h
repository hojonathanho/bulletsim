#pragma once
#include <btBulletDynamicsCommon.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <osg/Geometry>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include "clouds/pcl_typedefs.h"

inline btVector3 toBulletVector(const std::vector<float>& vec) {return btVector3(vec[0],vec[1],vec[2]);}
inline btVector3 toBulletVector(const Eigen::Vector3f& vec) {return btVector3(vec[0],vec[1],vec[2]);}
inline btVector3 toBulletVector(const osg::Vec3f &v) { return btVector3(v.x(), v.y(), v.z()); }
inline btVector3 toBulletVector(const geometry_msgs::Point& pt) { return btVector3(pt.x, pt.y, pt.z); }
inline btVector3 toBulletVector(const geometry_msgs::Point32& pt) { return btVector3(pt.x, pt.y, pt.z); }
inline Eigen::Vector3f toEigenVector(const std::vector<float>& vec) {return Eigen::Vector3f(vec[0],vec[1],vec[2]);}
inline Eigen::Vector3f toEigenVector(const btVector3& vec) {return Eigen::Vector3f(vec.x(),vec.y(),vec.z());}
inline Eigen::Vector3f toEigenVector(const ColorPoint& pt) {return Eigen::Vector3f(pt.x,pt.y,pt.z);}
inline osg::Vec3f toOSGVector(const btVector3 &v) { return osg::Vec3f(v.x(), v.y(), v.z()); }
Eigen::Matrix3f toEigenMatrix(const btMatrix3x3& basis);
btMatrix3x3 toBulletMatrix(const Eigen::Matrix3f& rotation);
osg::Matrix toOsgMatrix( const btTransform& t );
btTransform toBtTransform( const osg::Matrix& m );
std::vector<btVector3> toBulletVectors(const std::vector< std::vector<float> >& in);
std::vector<btVector3> toBulletVectors(const std::vector< Eigen::Vector3f >& in);
std::vector<btVector3> toBulletVectors(const Eigen::MatrixXf& in);
std::vector< std::vector<float> > toVecVec(const std::vector<btVector3>& in);
std::vector<float> toVec(const Eigen::VectorXf& in);
Eigen::VectorXf toVectorXf(const std::vector<float>& in);
btTransform toBulletTransform(const Eigen::Affine3f& affine);
btTransform toBulletTransform(const geometry_msgs::Transform&);
Eigen::Affine3f toEigenTransform(const btTransform& transform);
Eigen::Affine3f toEigenTransform(const geometry_msgs::Transform&);
geometry_msgs::Transform toROSTransform(const Eigen::Affine3f& in);
geometry_msgs::Transform toROSTransform(const btTransform& in);
std::vector<Eigen::Vector3f> toEigenVectors(const std::vector< std::vector<float> >& in);
std::vector<Eigen::Vector3f> toEigenVectors(const std::vector<btVector3>& in);
Eigen::MatrixXf toEigenMatrix(const std::vector<btVector3>& in);
Eigen::MatrixXf toEigenMatrix(const std::vector<Eigen::Vector3f>& in);
Eigen::MatrixXf toEigenMatrix(const std::vector< std::vector<float> >& in);
std::vector<btVector3> toBulletVectors(const std::vector<geometry_msgs::Point>&);
std::vector<btVector3> toBulletVectors(const std::vector<geometry_msgs::Point32>&);
std::vector<geometry_msgs::Point> toROSPoints(const std::vector<btVector3>&);
std::vector<geometry_msgs::Point32> toROSPoints32(const std::vector<btVector3>&);
ColorPoint toColorPoint(const Eigen::Vector3f& vec);
ColorPoint toColorPoint(const btVector3& vec);
Point toPoint(const Eigen::Vector3f& vec);
