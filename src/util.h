#ifndef __UTIL_H__
#define __UTIL_H__

#include <LinearMath/btTransform.h>
#include <osg/Vec3d>

namespace util {

// reads input from haptic devices (using getDeviceState),
// and then transforms the rotations/coordinates to our coordinate system
bool getHapticInput(btTransform &trans0, bool buttons0[2], btTransform &trans1, bool buttons1[2]);

inline btVector3 toBtVector(const osg::Vec3d &v) { return btVector3(v.x(), v.y(), v.z()); }

}

#endif // __UTIL_H__
