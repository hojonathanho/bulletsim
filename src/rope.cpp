#include "rope.h"
#include <iostream>
using namespace std;

CapsuleRope::CapsuleRope(const btAlignedObjectArray<btVector3>& ctrlPoints, btScalar radius_) {
  radius = radius_;
  nLinks = ctrlPoints.size() - 1;


  for (int i=0; i < nLinks; i++) {
    btVector3 pt0 = ctrlPoints[i];
    btVector3 pt1 = ctrlPoints[i+1];
    btVector3 midpt = (pt0+pt1)/2;
    btVector3 diff = (pt1-pt0);
    btQuaternion q;
    btScalar ang = diff.angle(btVector3(1,0,0));
    if (ang*ang > 1e-4) {
      btVector3 ax = diff.cross(btVector3(1,0,0));
      q = btQuaternion(ax,ang);
    }
    else {
      q = btQuaternion(0,0,0,1);
    }
    btTransform trans;
    trans.setOrigin(midpt);
    trans.setRotation(q);

    float len = diff.length();
    float mass = 1;//mass = 3.14*radius*radius*len;
    //btCollisionShape* shape = new btCapsuleShapeX(radius,len);
    shared_ptr<btCollisionShape> shapePtr(new btCylinderShapeX(btVector3(len/2,radius,radius)));

    bool isDynamic = (mass != 0.f);
    btVector3 localInertia(0,0,0);
    if (isDynamic) shapePtr->calculateLocalInertia(mass,localInertia);
    btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shapePtr.get(),localInertia);
    shared_ptr<btRigidBody> bodyPtr(new btRigidBody(cInfo));

    bodies.push_back(bodyPtr);

    BulletObject::Ptr child(new BulletObject(shapePtr,bodyPtr));
    children.push_back(child);

    if (i>0) {
      shared_ptr<btPoint2PointConstraint> jointPtr(new btPoint2PointConstraint(*bodies[i-1],*bodies[i],btVector3(len/2,0,0),btVector3(-len/2,0,0)));
      joints.push_back(jointPtr);
    }


  }
}

void CapsuleRope::init() {



  cout << "rope initializing" << endl;
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

