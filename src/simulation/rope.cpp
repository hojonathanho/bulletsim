#include "rope.h"
#include "basicobjects.h"
#include <iostream>
#include "bullet_io.h"
#include <boost/foreach.hpp>
using namespace std;

boost::shared_ptr<btGeneric6DofSpringConstraint> CapsuleRope_createBendConstraint(btScalar len,
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
btMatrix3x3 CapsuleRope_makePerpBasis(const btVector3& a0) {
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


void CapsuleRope_createRopeTransforms(vector<btTransform>& transforms, vector<btScalar>& lengths, const vector<btVector3>& ctrlPoints) {
  int nLinks = ctrlPoints.size()-1;
  for (int i=0; i < nLinks; i++) {
    btVector3 pt0 = ctrlPoints[i];
    btVector3 pt1 = ctrlPoints[i+1];
    btVector3 midpt = (pt0+pt1)/2;
    btVector3 diff = (pt1-pt0);

    btMatrix3x3 rotation = CapsuleRope_makePerpBasis(diff);


    btTransform trans(rotation,midpt);

    float len = diff.length();
    transforms.push_back(trans);
    lengths.push_back(len);
  }

}

vector<btVector3> CapsuleRope_getNodes(const vector<btRigidBody*> &capsules) { 
  vector<btVector3> out(capsules.size());
  for (int i=0; i < capsules.size(); i++)
    out[i] = capsules[i]->getCenterOfMassPosition();
  return out;
}

vector<btVector3> CapsuleRope_getControlPoints(const vector<btRigidBody*> &capsules) { 
  vector<btVector3> out;
  out.reserve(capsules.size()+1);
  for (int i=0; i < capsules.size(); i++) {
    btRigidBody* body = capsules[i];
    btCapsuleShape* capsule = dynamic_cast<btCapsuleShapeX*>(body->getCollisionShape());
    btTransform tf = body->getCenterOfMassTransform();
    if (i==0) out.push_back(tf.getOrigin() + btMatrix3x3(tf.getRotation())*btVector3(-capsule->getHalfHeight(),0,0));
    out.push_back(tf.getOrigin() + btMatrix3x3(tf.getRotation())*btVector3(capsule->getHalfHeight(),0,0));
  }
  return out;
}

vector<btMatrix3x3> CapsuleRope_getRotations(const vector<btRigidBody*> &capsules) {
  vector<btMatrix3x3> out;
  for (int i=0; i < capsules.size(); i++) {
    btRigidBody* body = capsules[i];
//    btCapsuleShape* capsule = dynamic_cast<btCapsuleShapeX*>(body->getCollisionShape());
    btTransform tf = body->getCenterOfMassTransform();
    out.push_back(tf.getBasis());
  }
  return out;
}

vector<float> CapsuleRope_getHalfHeights(const vector<btRigidBody*> &capsules) {
  vector<float> out;
  for (int i=0; i < capsules.size(); i++) {
    btRigidBody* body = capsules[i];
    btCapsuleShape* capsule = dynamic_cast<btCapsuleShapeX*>(body->getCollisionShape());
    out.push_back(capsule->getHalfHeight());
  }
  return out;
}

static vector<btRigidBody*> extractRigidBodies(const vector<BulletObject::Ptr> &children) {
  vector<btRigidBody*> out(children.size());
  for (int i = 0; i < children.size(); ++i) {
    out[i] = children[i]->rigidBody.get();
  }
  return out;
}


CapsuleRope::CapsuleRope(const vector<btVector3>& ctrlPoints, btScalar radius_, float angStiffness_, float angDamping_, float linDamping_, float angLimit_, float linStopErp_) {
  radius = radius_;
  angStiffness = angStiffness_;
  angDamping = angDamping_;
  linDamping = linDamping_;
  angLimit = angLimit_;
  nLinks = ctrlPoints.size()-1;
  vector<btTransform> transforms;
  vector<btScalar> lengths;
  CapsuleRope_createRopeTransforms(transforms,lengths,ctrlPoints);
  for (int i=0; i < nLinks; i++) {
    btTransform trans = transforms[i];
    btScalar len = lengths[i];
    float mass = 0.2;
    CapsuleObject::Ptr child(new CapsuleObject(mass,radius,len,trans));
    child->rigidBody->setDamping(linDamping,angDamping);
    child->rigidBody->setFriction(1);
    //child->collisionShape->setMargin(0.04);

    children.push_back(child);

    if (i>0) {
      boost::shared_ptr<btPoint2PointConstraint> jointPtr(new btPoint2PointConstraint(*children[i-1]->rigidBody,*children[i]->rigidBody,btVector3(len/2,0,0),btVector3(-len/2,0,0)));
      jointPtr->setParam(BT_CONSTRAINT_STOP_ERP, linStopErp_);
      joints.push_back(BulletConstraint::Ptr(new BulletConstraint(jointPtr, true)));

      boost::shared_ptr<btGeneric6DofSpringConstraint> springPtr = CapsuleRope_createBendConstraint(len,children[i-1]->rigidBody,children[i]->rigidBody,angDamping,angStiffness,angLimit);
      joints.push_back(BulletConstraint::Ptr(new BulletConstraint(springPtr, true)));
    }
  }
  children_rigidBodies = extractRigidBodies(children);
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
  for (int i = 0; i < joints.size(); i++) {
    getEnvironment()->removeConstraint(joints[i]);
  }
  CompoundObject<BulletObject>::destroy();
}

vector<btVector3> CapsuleRope::getNodes() {
  return CapsuleRope_getNodes(children_rigidBodies);
}

vector<btVector3> CapsuleRope::getControlPoints() {
  return CapsuleRope_getControlPoints(children_rigidBodies);
}

vector<btMatrix3x3> CapsuleRope::getRotations() {
  return CapsuleRope_getRotations(children_rigidBodies);
}

vector<float> CapsuleRope::getHalfHeights() {
  return CapsuleRope_getHalfHeights(children_rigidBodies);
}
