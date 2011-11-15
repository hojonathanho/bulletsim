#pragma once
#include "btBulletDynamicsCommon.h"

class Grab
{
public:
  btGeneric6DofConstraint* cnt;
  Grab(){}
  Grab(btRigidBody* rb, btVector3 pos, btDynamicsWorld* world) {
    cnt = new btGeneric6DofConstraint(*rb,btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)),true); // second parameter?
    cnt->setLinearLowerLimit(btVector3(0,0,0));
    cnt->setLinearUpperLimit(btVector3(0,0,0));
    cnt->setAngularLowerLimit(btVector3(0,0,0));
    cnt->setAngularUpperLimit(btVector3(0,0,0));
    world->addConstraint(cnt);
    updatePosition(pos);
  }

  void updatePosition(btVector3 pos) {
    cnt->getFrameOffsetA().setOrigin(pos);
  }
};
