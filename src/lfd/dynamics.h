#ifndef _LFD_DYNAMICS_H_
#define _LFD_DYNAMICS_H_

#include "lfd_rope_common.h"

namespace lfd {

struct RopeRobotSystem {
  typedef boost::shared_ptr<RopeRobotSystem> Ptr;
  ~RopeRobotSystem();

  Environment::Ptr env;
  RaveInstancePtr rave;

  RaveRobotObject::Ptr robot;
  RaveRobotObject::Manipulator::Ptr manip;
  CapsuleRope::Ptr rope;

  class ScopedLock {
  public:
    typedef boost::shared_ptr<ScopedLock> Ptr;
    ScopedLock(RaveRobotObject::Manipulator::Ptr manip_);
    ~ScopedLock();
  private:
    vector<double> origManipDofs;
    RaveRobotObject::Manipulator::Ptr manip;
  };
  ScopedLock::Ptr lock();

  // for debugging
  Scene *scene;
  void enableDrawing(Scene *);
  void enableDrawing();
  void draw();
  void disableDrawing();

  Ptr fork() const;

  static Ptr InitFrom(const LFDRopeScene &);

  void assertIntegrity();
};


} // namespace lfd


#endif // _LFD_DYNAMICS_H_
