#pragma once
#include <btBulletDynamicsCommon.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

inline btVector3 toBulletVector(const std::vector<float>& vec) {return btVector3(vec[0],vec[1],vec[2]);}
inline btVector3 toBulletVector(const Eigen::Vector3f& vec) {return btVector3(vec[0],vec[1],vec[2]);}
inline Eigen::Vector3f toEigenVector(const std::vector<float>& vec) {return Eigen::Vector3f(vec[0],vec[1],vec[2]);}
inline Eigen::Vector3f toEigenVector(const btVector3& vec) {return Eigen::Vector3f(vec.x(),vec.y(),vec.z());}
Eigen::Matrix3f toEigenMatrix(const btMatrix3x3& basis);
btMatrix3x3 toBulletMatrix(const Eigen::Matrix3f& rotation);
std::vector<btVector3> toBulletVectors(const std::vector< std::vector<float> >& in);
std::vector<btVector3> toBulletVectors(const std::vector< Eigen::Vector3f >& in);
std::vector<btVector3> toBulletVectors(const Eigen::MatrixXf& in);
std::vector< std::vector<float> > toVecVec(const std::vector<btVector3>& in);
std::vector<float> toVec(const Eigen::VectorXf& in);
Eigen::VectorXf toVectorXf(const std::vector<float>& in);
btTransform toBulletTransform(const Eigen::Affine3f& affine);
Eigen::Affine3f toEigenTransform(const btTransform& transform);
std::vector<Eigen::Vector3f> toEigenVectors(const std::vector< std::vector<float> >& in);
std::vector<Eigen::Vector3f> toEigenVectors(const std::vector<btVector3>& in);
Eigen::MatrixXf toEigenMatrix(const std::vector<btVector3>& in);
Eigen::MatrixXf toEigenMatrix(const std::vector<Eigen::Vector3f>& in);
Eigen::MatrixXf toEigenMatrix(const std::vector< std::vector<float> >& in);
