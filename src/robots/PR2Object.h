/*
 * PR2Object.h
 *
 *  Created on: Nov 16, 2012
 *      Author: alex
 */

#ifndef PR2OBJECT_H_
#define PR2OBJECT_H_

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "robots/pr2.h"
#include "robots/grabbing.h"
#include "robots/grab_detection.h"
#include "simulation/openravesupport.h"
#include <sensor_msgs/JointState.h>

class PR2Object : public RaveRobotObject, public boost::enable_shared_from_this<PR2Object> {
private:
	PR2Object(); //Only for copying
  vector<Manipulator::Ptr> m_manipulators;
  vector<Monitor::Ptr> m_monitors;
  vector<GrabDetector::Ptr> m_detectors;

protected:
  void init();
  EnvironmentObject::Ptr copy(Fork &f) const;
  void postCopy(EnvironmentObject::Ptr copy, Fork &f) const;

public:
  typedef boost::shared_ptr<PR2Object> Ptr;

  enum ManipId {
  	ALL = -1,
  	LEFT,
  	RIGHT,
  	MANIP_ID_SIZE
  };
  vector<ManipId> m_manip_ids; // possible ManidId

  PR2Object(RaveInstance::Ptr rave);

  void drive(float dx, float dy, float da) { setTransform(getTransform() * btTransform(btQuaternion(0,0,da), btVector3(dx*METERS, dy*METERS, 0))); }

  void setTarget(BulletSoftObject::Ptr sb); // Need to specify softbody that the pr2 is about to grab. TODO the pr2 should consider all the softbodies in the environment
  void grab(ManipId manip_id = ALL);
  void release(ManipId manip_id = ALL);

  void setJointState(const sensor_msgs::JointState& msg); // If the grippers are closing and if the target softbody is nearby, then the pr2 grabs the softbody
};

bool inline operator==(PR2Object::ManipId lhs, PR2Object::ManipId rhs) {
	return ((int) lhs) == ((int) rhs);
}

#endif /* PR2OBJECT_H_ */



