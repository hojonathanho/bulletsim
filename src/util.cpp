#include "util.h"
#include "thread_socket_interface.h"

namespace util {

static const btScalar HAPTIC_TRANS_SCALE = 1./40.;
static const btVector3 HAPTIC_OFFSET0(0., -2., 1.);
static const btVector3 HAPTIC_OFFSET1(0., 2., 1.);
static const btMatrix3x3 HAPTIC_ROTATION(btQuaternion(-M_PI/2., 0., 0.));
static inline btMatrix3x3 toHapticBtMatrix(const Matrix3d &m) {
    // note: the rows are permuted
    return btMatrix3x3(m(2, 0), m(2, 1), m(2, 2),
                       m(0, 0), m(0, 1), m(0, 2),
                       m(1, 0), m(1, 1), m(1, 2));
}
static inline btVector3 toHapticBtVector(const Vector3d &v) {
    // note: the components are permuted
    return btVector3(v.z(), v.x(), v.y());
}
bool getHapticInput(btTransform &trans0, bool buttons0[2], btTransform &trans1, bool buttons1[2]) {
    Vector3d start_proxy_pos, end_proxy_pos;
    Matrix3d start_proxy_rot, end_proxy_rot;
    if (!getDeviceState(start_proxy_pos, start_proxy_rot, buttons0,
                        end_proxy_pos, end_proxy_rot, buttons1))
        return false;
    trans0 = btTransform(toHapticBtMatrix(start_proxy_rot) * HAPTIC_ROTATION,
                         HAPTIC_TRANS_SCALE*toHapticBtVector(start_proxy_pos) + HAPTIC_OFFSET0);
    trans1 = btTransform(toHapticBtMatrix(end_proxy_rot) * HAPTIC_ROTATION,
                         HAPTIC_TRANS_SCALE*toHapticBtVector(end_proxy_pos) + HAPTIC_OFFSET1);
    return true;
}


}
