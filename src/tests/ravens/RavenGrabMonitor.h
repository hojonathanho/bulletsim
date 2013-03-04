#pragma once
#include "robots/grabbing.h"
#include "simulation/util.h"
#include <openrave/kinbody.h>
#include "simulation/simplescene.h"


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
public:
  typedef boost::shared_ptr<RavensGrabMonitor> Ptr;

  // min/max gripper dof vals
  static const float CLOSED_VAL = 0.01f, OPEN_VAL = 0.25f;

  KinBody::LinkPtr leftFinger, rightFinger;
  const btTransform origLeftFingerInvTrans, origRightFingerInvTrans;
  const btVector3 centerPt;
  vector<int> indices;

  std::vector<BulletObject::Ptr> m_bodies;
  std::vector<int> m_collidingBodies;
  btDynamicsWorld* m_world;
  std::vector<RavensGrab::Ptr> m_grabs;
  int m_i;
  int numGrabbed;
  Scene &s;


  RavensGrabMonitor(RaveRobotObject::Manipulator::Ptr _manip, btDynamicsWorld* _world,
		  	  	  	  const string &leftFingerName, const string &rightFingerName, Scene &s);
  btTransform getManipRot() const;
  btTransform getInverseFingerTfm (bool left);
  btVector3 getInnerPt(bool left) const ;
  btVector3 getClosingDirection(bool left) const;
  btVector3 getToolDirection() const;
  bool onInnerSide(const btVector3 &pt, bool left);

  bool checkContacts (bool leftFinger, btRigidBody *target, float threshold);
  void setBodies(std::vector<BulletObject::Ptr>& bodies) {m_bodies = bodies;}
  void grab();
  void grab(float threshold);
  int getNumGrabbed() {return numGrabbed;}
  void release();
  void updateGrabPose();
  float getGripperAngle(RaveRobotObject::Manipulator::Ptr manip);
  bool isClosed(RaveRobotObject::Manipulator::Ptr manip, float closedThreshold);
};




