#ifndef __UTIL_H__
#define __UTIL_H__

#include <LinearMath/btTransform.h>

namespace util {

// reads input from haptic devices (using getDeviceState),
// and then transforms the rotations/coordinates to our coordinate system
bool getHapticInput(btTransform &trans0, bool buttons0[2], btTransform &trans1, bool buttons1[2]);

}

#endif // __UTIL_H__
