#include "rope.h"
#include "basicobjects.h"
#include <iostream>
using namespace std;

shared_ptr<btRigidBody> createRigidBody(const shared_ptr<btCollisionShape> shapePtr, const btTransform& trans, btScalar mass) {
  bool isDynamic = (mass != 0.f);
  btVector3 localInertia(0,0,0);
  if (isDynamic) shapePtr->calculateLocalInertia(mass,localInertia);
  btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
  btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shapePtr.get(),localInertia);
  return shared_ptr<btRigidBody>(new btRigidBody(cInfo));
}

shared_ptr<btGeneric6DofSpringConstraint>  createBendConstraint(btScalar len,
			  const shared_ptr<btRigidBody> rbA, const shared_ptr<btRigidBody>& rbB, float damping, float stiffness, float limit) {

  btTransform tA,tB;
  tA.setIdentity(); tB.setIdentity();
  tA.setOrigin(btVector3(len/2,0,0)); tB.setOrigin(btVector3(-len/2,0,0));
  shared_ptr<btGeneric6DofSpringConstraint> springPtr = shared_ptr<btGeneric6DofSpringConstraint>(new btGeneric6DofSpringConstraint(*rbA,*rbB,tA,tB,false));
  for (int i=3; i<=5; i++) {
    springPtr->enableSpring(i,true);
    springPtr->setStiffness(i,stiffness);
    springPtr->setDamping(i,damping);
  }
  springPtr->setAngularLowerLimit(btVector3(-limit,-limit,-limit));
  springPtr->setAngularUpperLimit(btVector3(limit,limit,limit));
  return springPtr;
}

void createRopeTransforms(vector<btTransform>& transforms, vector<btScalar>& lengths, const vector<btVector3>& ctrlPoints) {
  int nLinks = ctrlPoints.size()-1;
  for (int i=0; i < nLinks; i++) {
    btVector3 pt0 = ctrlPoints[i];
    btVector3 pt1 = ctrlPoints[i+1];
    btVector3 midpt = (pt0+pt1)/2;
    btVector3 diff = (pt1-pt0);
    btQuaternion q;
    btScalar ang = diff.angle(btVector3(1,0,0));
    if (ang*ang > 1e-4) {
      btVector3 ax = diff.cross(btVector3(1,0,0));
      q = btQuaternion(ax,-ang);
    }
    else {
      q = btQuaternion(0,0,0,1);
    }
    btTransform trans(q,midpt);

    float len = diff.length();
    transforms.push_back(trans);
      lengths.push_back(len);
  }
}


CapsuleRope::CapsuleRope(const vector<btVector3>& ctrlPoints, btScalar radius_, float angStiffness_, float angDamping_, float linDamping_, float angLimit_) {
  radius = radius_;
  angStiffness = angStiffness_;
  angDamping = angDamping_;
  linDamping = linDamping_;
  angLimit = angLimit_;
  nLinks = ctrlPoints.size()-1;
  vector<btTransform> transforms;
  vector<btScalar> lengths;
  createRopeTransforms(transforms,lengths,ctrlPoints);
  for (int i=0; i < nLinks; i++) {
    btTransform trans = transforms[i];
    btScalar len = lengths[i];
    float mass = 1;
    CapsuleObject::Ptr child(new CapsuleObject(1,radius,len,trans));
    child->rigidBody->setDamping(linDamping,angDamping);
    child->rigidBody->setFriction(1);

    children.push_back(child);

    if (i>0) {
      shared_ptr<btPoint2PointConstraint> jointPtr(new btPoint2PointConstraint(*children[i-1]->rigidBody,*children[i]->rigidBody,btVector3(len/2,0,0),btVector3(-len/2,0,0)));
      joints.push_back(BulletConstraint::Ptr(new BulletConstraint(jointPtr, true)));

      shared_ptr<btGeneric6DofSpringConstraint> springPtr = createBendConstraint(len,children[i-1]->rigidBody,children[i]->rigidBody,angDamping,angStiffness,angLimit);
      joints.push_back(BulletConstraint::Ptr(new BulletConstraint(springPtr, true)));
    }
  }
}

void CapsuleRope::init() {

  CompoundObject<BulletObject>::init();



   // for (int i=0; i < children.size()-1; i++) {
   //        BulletObject::Ptr bo0 = static_cast<BulletObject::Ptr>(children[i]);
   //   BulletObject::Ptr bo1 = static_cast<BulletObject::Ptr>(children[i+1]);

   //   btRigidBody* body0 = (bo0->rigidBody).get();
     //btRigidBody* body1 = (bo1->rigidBody).get();
     //         btPoint2PointConstraint* joint = new btPoint2PointConstraint(*body0,*body1,btVector3(.1/2,0,0),btVector3(-.1/2,0,0));
  //      getEnvironment()->bullet->dynamicsWorld->addConstraint(joint);
  //  }


  for (int i=0; i< joints.size(); i++) {
       getEnvironment()->addConstraint(joints[i]);
  }
}

void CapsuleRope::destroy() {
/*  for (int i = 0; i < joints.size(); i++) {
       getEnvironment()->bullet->dynamicsWorld->removeConstraint(joints[i].get());
  }*/
  CompoundObject<BulletObject>::destroy();
}

