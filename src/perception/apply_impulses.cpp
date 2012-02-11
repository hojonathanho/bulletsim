#include "apply_impulses.h"

void applyImpulses(const vector<btVector3>& impulses, CapsuleRope::Ptr rope) {
  for (int i=0; i<rope->bodies.size(); i++) rope->bodies[i]->applyCentralImpulse(impulses[i]);
}

void applyImpulses(const vector<btVector3>& impulses, BulletSoftObject::Ptr cloth) {
  for (int i=0; i<impulses.size(); i++)
    cloth->softBody->addForce(impulses[i],i);
}
