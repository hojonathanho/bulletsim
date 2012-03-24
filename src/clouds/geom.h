#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <LinearMath/btTransform.h>

// void perpBasis(const Wm5::Vector3f& v1, Wm5::Vector3f& v2, Wm5::Vector3f& v3);
// float angBetween(const Wm5::Vector3f& v1, const Wm5::Vector3f& v2);

// void minEncRect(const std::vector<Wm5::Vector3f>& pts3d, const Wm5::Vector4f& abcd, std::vector<Wm5::Vector3f>& verts3d);
void minEncRect(const std::vector<Eigen::Vector3f>& pts3d, const Eigen::Vector4f& abcd, std::vector<Eigen::Vector3f>& verts3d);

// void minRot(const Wm5::Vector3f& v1, const Wm5::Vector3f& v2, Wm5::Matrix3f m);
void minRot(const btVector3& v1, const btVector3& v2, btMatrix3x3& m);
void minRot(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, Eigen::Matrix3f& m);

std::vector<Eigen::Vector3f>  getCorners(const std::vector<Eigen::Vector3f>& pts);

Eigen::Affine3f camToWorldFromTable(const std::vector< std::vector<float> >& corners);
