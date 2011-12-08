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


CapsuleRope::CapsuleRope(const vector<btVector3>& ctrlPoints, btScalar radius_, float stiffness_, float damping_, float limit_) {
  radius = radius_;
  stiffness = stiffness_;
  damping = damping_;
  limit = limit_;
  int nLinks = ctrlPoints.size()-1;
  vector<btTransform> transforms;
  vector<btScalar> lengths;
  createRopeTransforms(transforms,lengths,ctrlPoints);
  for (int i=0; i < nLinks; i++) {
    btTransform trans = transforms[i];
    btScalar len = lengths[i];
    float mass = 1;//mass = 3.14*radius*radius*len;
    //btCollisionShape* shape = new btCapsuleShapeX(radius,len);
    
    /*
    shared_ptr<btCollisionShape> shapePtr(new btCylinderShapeX(btVector3(len/2,radius,radius)));
    shared_ptr<btRigidBody> bodyPtr;
    createRigidBody(bodyPtr,shapePtr,trans,mass);
    bodyPtr->setFriction(10);
    bodies.push_back(bodyPtr);
    BulletObject::Ptr child(new BulletObject(shapePtr,bodyPtr));
    */
    shared_ptr<btDefaultMotionState> ms(new btDefaultMotionState(trans));

    CapsuleObject::Ptr child(new CapsuleObject(1,radius,len,ms));
    bodies.push_back(child->rigidBody);
    child->rigidBody->setDamping(.75,1);
    child->rigidBody->setFriction(1);

    children.push_back(child);

    if (i>0) {
      shared_ptr<btPoint2PointConstraint> jointPtr(new btPoint2PointConstraint(*bodies[i-1],*bodies[i],btVector3(len/2,0,0),btVector3(-len/2,0,0)));
      joints.push_back(jointPtr);


      shared_ptr<btGeneric6DofSpringConstraint> springPtr = createBendConstraint(len,bodies[i-1],bodies[i],damping,stiffness,limit);
      joints.push_back(springPtr);
							  
    }


  }
}

void CapsuleRope::init() {

  CompoundObject::init();



   // for (int i=0; i < children.size()-1; i++) {
   //        BulletObject::Ptr bo0 = static_cast<BulletObject::Ptr>(children[i]);
   //   BulletObject::Ptr bo1 = static_cast<BulletObject::Ptr>(children[i+1]);

   //   btRigidBody* body0 = (bo0->rigidBody).get();
     //btRigidBody* body1 = (bo1->rigidBody).get();
     //         btPoint2PointConstraint* joint = new btPoint2PointConstraint(*body0,*body1,btVector3(.1/2,0,0),btVector3(-.1/2,0,0));
  //      getEnvironment()->bullet->dynamicsWorld->addConstraint(joint);
  //  }


  for (int i=0; i< joints.size(); i++) {
       getEnvironment()->bullet->dynamicsWorld->addConstraint(joints[i].get(),true);
  }
}

void CapsuleRope::destroy() {
  for (int i = 0; i < joints.size(); i++) {
       getEnvironment()->bullet->dynamicsWorld->removeConstraint(joints[i].get());
  }
  CompoundObject<BulletObject>::destroy();
}

