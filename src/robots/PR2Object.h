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
  vector<HysterisGrabDetector::Ptr> m_detectors;
  void update(); // should be called every time the joint state of the robot is changed

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
  typedef boost::function<void(ManipId)> ManipCallback;

private:
  vector<ManipCallback> preGrabCallbacks;
	vector<ManipCallback> preReleaseCallbacks;
	vector<ManipCallback> postStateCallbacks;

public:
  PR2Object(RaveInstance::Ptr rave);

  void drive(float dx, float dy, float da) { setTransform(getTransform() * btTransform(btQuaternion(0,0,da), btVector3(dx*METERS, dy*METERS, 0))); }
  void moveByIK(ManipId manip_id, float angle_axis, float finger_normal_axis, float wrist_axis) {
		m_manipulators[manip_id]->moveByIK(m_manipulators[manip_id]->getTransform() * btTransform(btQuaternion(0,0,0), btVector3(angle_axis, finger_normal_axis, wrist_axis)*METERS));
  }
  float getGripperAngle(ManipId manip_id) { return m_manipulators[manip_id]->getGripperAngle(); }
  void setGripperAngle(ManipId manip_id, float angle) {
  	m_manipulators[manip_id]->setGripperAngle(angle);
  	update();
  }
  void changeGripperAngle(ManipId manip_id, float delta) {
  	setGripperAngle(manip_id, getGripperAngle(manip_id) + delta);
  }

  void grab(ManipId manip_id = ALL);
  void release(ManipId manip_id = ALL);
  void setJointState(const sensor_msgs::JointState& msg); // If the grippers are closing and if the target softbody is nearby, then the pr2 grabs the softbody
  string associatedLinkName(const string& joint_name);

	void addPreGrabCallback(ManipCallback cb) { preGrabCallbacks.push_back(cb); }
	void addPreReleaseCallback(ManipCallback cb) { preReleaseCallbacks.push_back(cb); }
	void addPostStateCallback(ManipCallback cb) { postStateCallbacks.push_back(cb); }
};

bool inline operator==(PR2Object::ManipId lhs, PR2Object::ManipId rhs) {
	return ((int) lhs) == ((int) rhs);
}

#endif /* PR2OBJECT_H_ */



