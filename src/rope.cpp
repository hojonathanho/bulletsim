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
    if (ang > 0) {
      btVector3 ax = diff.cross(btVector3(1,0,0));
      q = btQuaternion(ax,ang);
    }
    else {
      q = btQuaternion();
    }
    btTransform trans;
    trans.setOrigin(midpt);
    trans.setRotation(q);

    float len = diff.length();
    float mass = mass = 3.14*radius*radius*len;
    //btCollisionShape* shape = new btCapsuleShapeX(radius,len);
    btCollisionShape* shape = new btCylinderShapeX(btVector3(len,radius,radius));

    bool isDynamic = (mass != 0.f);
    btVector3 localInertia(0,0,0);
    if (isDynamic) shape->calculateLocalInertia(mass,localInertia);
    btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);
    btRigidBody* body = new btRigidBody(cInfo);

    cout << body->getCenterOfMassPosition().x() << endl;
    bodies.push_back(body);

    boost::shared_ptr<btCollisionShape> shapePtr;
    boost::shared_ptr<btMotionState> msPtr;
    boost::shared_ptr<btRigidBody> rbPtr;
    shapePtr.reset(shape);
    msPtr.reset(myMotionState);
    rbPtr.reset(body);
    BulletObject::Ptr child(new BulletObject(shapePtr,msPtr,rbPtr));
    children.push_back(child);


    if (i>0) {
      btPoint2PointConstraint* joint = new btPoint2PointConstraint(*bodies[i-1],*body,btVector3(-len/2,0,0),btVector3(len/2,0,0));
      joints.push_back(joint);
    }
  }
}

void CapsuleRope::init() {
  cout << "rope initializing" << endl;
  CompoundObject::init();

  for (int i=0; i< joints.size(); i++) {
    btTypedConstraint* joint = joints[i];
    //getEnvironment()->bullet->dynamicsWorld->addConstraint(joint,true);
  }
}

