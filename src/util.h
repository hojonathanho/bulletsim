#ifndef __UTIL_H__
#define __UTIL_H__

#include <LinearMath/btTransform.h>
#include <osg/Vec3d>
#include <openrave/openrave.h>

namespace util {

// reads input from haptic devices (using getDeviceState),
// and then transforms the rotations/coordinates to our coordinate system
bool getHapticInput(btTransform &trans0, bool buttons0[2], btTransform &trans1, bool buttons1[2]);

inline osg::Vec3d toOSGVector(const btVector3 &v) { return osg::Vec3d(v.x(), v.y(), v.z()); }

inline btVector3 toBtVector(const osg::Vec3d &v) { return btVector3(v.x(), v.y(), v.z()); }
inline btVector3 toBtVector(const OpenRAVE::Vector &v) { return btVector3(v.x, v.y, v.z); }
inline btTransform toBtTransform(const OpenRAVE::Transform &t) {
    return btTransform(btQuaternion(t.rot.y, t.rot.z, t.rot.w, t.rot.x), toBtVector(t.trans));
}
inline btTransform toBtTransform(const OpenRAVE::Transform &t, btScalar scale) {
    return btTransform(btQuaternion(t.rot.y, t.rot.z, t.rot.w, t.rot.x), scale * toBtVector(t.trans));
}

inline OpenRAVE::Vector toRaveQuaternion(const btQuaternion &q) {
    return OpenRAVE::Vector(q.w(), q.x(), q.y(), q.z());
}
inline OpenRAVE::Vector toRaveVector(const btVector3 &v) {
    return OpenRAVE::Vector(v.x(), v.y(), v.z());
}
inline OpenRAVE::Transform toRaveTransform(const btTransform &t) {
    return OpenRAVE::Transform(toRaveQuaternion(t.getRotation()), toRaveVector(t.getOrigin()));
}
inline OpenRAVE::Transform toRaveTransform(const btTransform &t, btScalar scale) {
    return OpenRAVE::Transform(toRaveQuaternion(t.getRotation()), toRaveVector(scale * t.getOrigin()));
}

}

#endif // __UTIL_H__
