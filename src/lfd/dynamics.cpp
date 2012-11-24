#include "dynamics.h"
#include "lfd_rope_common.h"

namespace lfd {

RopeRobotSystem::Ptr RopeRobotSystem::InitFrom(const LFDRopeScene &s) {
  RopeRobotSystem::Ptr sys(new RopeRobotSystem);
  sys->env = s.scene->env;
  sys->manip = s.scene->pr2m->pr2Left;
  sys->robot = s.scene->pr2m->pr2;
  sys->rope = s.scene->m_rope;
  sys->scene = NULL;
  sys->rave = s.scene->pr2m->pr2->rave;
  sys->assertIntegrity();
  return sys->fork();
//  return sys;
}

RopeRobotSystem::Ptr RopeRobotSystem::fork() const {
  Fork::Ptr fork(new Fork(env, rave));

  RaveRobotObject::Ptr fork_robot =
    boost::static_pointer_cast<RaveRobotObject>(fork->forkOf(robot));
  CapsuleRope::Ptr fork_rope =
    boost::static_pointer_cast<CapsuleRope>(fork->forkOf(rope));
  RaveRobotObject::Manipulator::Ptr fork_manip =
    fork_robot->getManipByIndex(manip->index);

  RopeRobotSystem::Ptr sys(new RopeRobotSystem);
  sys->env = fork->env;
  sys->manip = fork_manip;
  sys->robot = fork_robot;
  sys->rope = fork_rope;
  sys->scene = scene;
  sys->rave = rave;
  sys->assertIntegrity();
  return sys;
}

void RopeRobotSystem::enableDrawing(Scene *s) {
  scene = s;
  enableDrawing();
}

void RopeRobotSystem::enableDrawing() {
  /*if (scene) {
    scene->setup(env);
  }*/
  if (scene && !scene->osg->root->containsNode(env->osg->root.get())) {
    env->preDraw();
    scene->osg->root->addChild(env->osg->root.get());
  }
}

void RopeRobotSystem::draw() {
  if (scene) {
    scene->draw();
  }
}

void RopeRobotSystem::disableDrawing() {
  if (scene) {
    scene->osg->root->removeChild(env->osg->root.get());
  }
  scene = NULL;
}

RopeRobotSystem::~RopeRobotSystem() {
  disableDrawing();
}

void RopeRobotSystem::assertIntegrity() {
  assert(env);
  assert(robot);
  assert(manip);
  assert(rope);
}

RopeRobotSystem::ScopedLock::ScopedLock(RaveRobotObject::Manipulator::Ptr manip_) :
  manip(manip_),
  origManipDofs(manip_->getDOFValues())
{
}

RopeRobotSystem::ScopedLock::~ScopedLock() {
  manip->setDOFValues(origManipDofs);
}

RopeRobotSystem::ScopedLock::Ptr RopeRobotSystem::lock() {
  return ScopedLock::Ptr(new ScopedLock(manip));
}

} // namespace lfd
