#pragma once
#include "robots/grabbing.h"
#include "simulation/util.h"
#include <openrave/kinbody.h>
#include "simulation/simplescene.h"

class CustomScene;

class RavensGrab : public Grab {

public:

	typedef boost::shared_ptr<RavensGrab> Ptr;

	// offset of the center of mass frame of the body from the finger at the time of contact
	btTransform offset;

	// True iff left finger of the end-effector is grasping
	bool leftFinger;

	RavensGrab(btRigidBody* rb, const btTransform& pose,
				btDynamicsWorld* world, bool leftFinger_, btTransform &offset_);

};


class RavensGrabMonitor : public Monitor {

	void testingg(bool);

public:
  typedef boost::shared_ptr<RavensGrabMonitor> Ptr;

  // min/max gripper dof vals
  static const float CLOSED_VAL = 0.01f, OPEN_VAL = 0.25f;

  RaveRobotObject::Manipulator::Ptr manip;
  char gripper;

  KinBody::LinkPtr leftFinger, rightFinger;
  const btTransform origLeftFingerInvTrans, origRightFingerInvTrans;
  const btVector3 centerPt;
  vector<int> indices;

  std::vector<CompoundObject<BulletObject>::Ptr> m_bodies; // changed so that we can use the moving average over a compound object while grabbing
  std::vector<int> m_collidingBodies;
  btDynamicsWorld* m_world;
  std::vector<RavensGrab::Ptr> m_grabs;
  int m_i;
  int numGrabbed;
  CustomScene &s;


  RavensGrabMonitor(RaveRobotObject::Manipulator::Ptr _manip, btDynamicsWorld* _world, char _gripper,
		  	  	  	  const string &leftFingerName, const string &rightFingerName, CustomScene &s);
  btTransform getManipRot() const;
  btTransform getInverseFingerTfm (bool left);
  btVector3 getInnerPt(bool left) const ;
  btVector3 getClosingDirection(bool left) const;
  btVector3 getToolDirection() const;
  bool onInnerSide(const btVector3 &pt, bool left, btVector3 threshVec=btVector3(0.0025, -0.001, 0.005));

  bool checkContacts (bool leftFinger, btRigidBody *target, double &avg_impulse, float threshold=100.f,
		  	  	  	  btVector3 grabThreshVec=btVector3(0.0025, -0.001, 0.005));
  void setBodies(std::vector<CompoundObject<BulletObject>::Ptr>& bodies) {m_bodies = bodies;}
  void grab();
  void grab(bool grabN, float threshold=100);
  void grabNeedle();
  int  getNumGrabbed() {return numGrabbed;}
  void release();
  void updateGrabPose();
  float getGripperAngle(RaveRobotObject::Manipulator::Ptr manip);
  bool  isClosed(RaveRobotObject::Manipulator::Ptr manip, float closedThreshold);
};




