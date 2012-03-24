#include "rope.h"
#include "basicobjects.h"
#include <iostream>
#include "bullet_io.h"
using namespace std;

boost::shared_ptr<btRigidBody> createRigidBody(const boost::shared_ptr<btCollisionShape> shapePtr, const btTransform& trans, btScalar mass) {
  bool isDynamic = (mass != 0.f);
  btVector3 localInertia(0,0,0);
  if (isDynamic) shapePtr->calculateLocalInertia(mass,localInertia);
  btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
  btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shapePtr.get(),localInertia);
  return boost::shared_ptr<btRigidBody>(new btRigidBody(cInfo));
}

boost::shared_ptr<btGeneric6DofSpringConstraint>  createBendConstraint(btScalar len,
								       const boost::shared_ptr<btRigidBody> rbA, const boost::shared_ptr<btRigidBody>& rbB, float damping, float stiffness, float limit) {

  btTransform tA,tB;
  tA.setIdentity(); tB.setIdentity();
  tA.setOrigin(btVector3(len/2,0,0)); tB.setOrigin(btVector3(-len/2,0,0));
  boost::shared_ptr<btGeneric6DofSpringConstraint> springPtr = boost::shared_ptr<btGeneric6DofSpringConstraint>(new btGeneric6DofSpringConstraint(*rbA,*rbB,tA,tB,false));
  for (int i=3; i<=5; i++) {
    springPtr->enableSpring(i,true);
    springPtr->setStiffness(i,stiffness);
    springPtr->setDamping(i,damping);
  }
  springPtr->setAngularLowerLimit(btVector3(-limit,-limit,-limit));
  springPtr->setAngularUpperLimit(btVector3(limit,limit,limit));
  return springPtr;
}

const float SMALL_FLOAT = 1e-7;
static btMatrix3x3 makePerpBasis(const btVector3& a0) {
  btVector3 a = a0.normalized();
  if ((a.x() == 0) && (a.y() == 0)) {
    return btMatrix3x3(0,0,1,0,1,0,0,0,1);
  }
  else {
    btVector3 b = btVector3(a0.y(), -a0.x(), 0);
    b.normalize();
    btVector3 c = a.cross(b);
    return btMatrix3x3(a.x(), a.y(), a.z(),
		       b.x(), b.y(), b.z(),
		       c.x(), c.y(), c.z());
  }
}


void createRopeTransforms(vector<btTransform>& transforms, vector<btScalar>& lengths, const vector<btVector3>& ctrlPoints) {
  int nLinks = ctrlPoints.size()-1;
  for (int i=0; i < nLinks; i++) {
    btVector3 pt0 = ctrlPoints[i];
    btVector3 pt1 = ctrlPoints[i+1];
    btVector3 midpt = (pt0+pt1)/2;
    btVector3 diff = (pt1-pt0);

    btMatrix3x3 rotation = makePerpBasis(diff);


    btTransform trans(rotation,midpt);

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
      boost::shared_ptr<btPoint2PointConstraint> jointPtr(new btPoint2PointConstraint(*children[i-1]->rigidBody,*children[i]->rigidBody,btVector3(len/2,0,0),btVector3(-len/2,0,0)));
      joints.push_back(BulletConstraint::Ptr(new BulletConstraint(jointPtr, true)));

      boost::shared_ptr<btGeneric6DofSpringConstraint> springPtr = createBendConstraint(len,children[i-1]->rigidBody,children[i]->rigidBody,angDamping,angStiffness,angLimit);
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

vector<btVector3> CapsuleRope::getNodes() { 
  vector<btVector3> out(children.size());
  for (int i=0; i < children.size(); i++)
    out[i] = children[i]->rigidBody->getCenterOfMassPosition();
  return out;
}

vector<btVector3> CapsuleRope::getControlPoints() { 
  vector<btVector3> out;
  out.reserve(children.size()+1);
  for (int i=0; i < children.size(); i++) {
    btRigidBody* body = children[i]->rigidBody.get();
    btCapsuleShape* capsule = dynamic_cast<btCapsuleShapeX*>(body->getCollisionShape());
    btTransform tf = body->getCenterOfMassTransform();
    if (i==0) out.push_back(tf.getOrigin() + btMatrix3x3(tf.getRotation())*btVector3(-capsule->getHalfHeight(),0,0));
    out.push_back(tf.getOrigin() + btMatrix3x3(tf.getRotation())*btVector3(capsule->getHalfHeight(),0,0));
  }
  return out;
}
