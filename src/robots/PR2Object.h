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
#include "utils/utils_vector.h"

class PR2Object : public RaveRobotObject, public boost::enable_shared_from_this<PR2Object> {
private:
	PR2Object(); //Only for copying
  vector<Manipulator::Ptr> m_manipulators;
  vector<Monitor::Ptr> m_monitors;
  vector<HysterisGrabDetector::Ptr> m_detectors;
  void update(); // should be called every time the joint state of the robot is changed
  bool m_disable_set_joint_state;

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
  static const vector<ManipId> m_manip_ids; // possible ManidId
  typedef boost::function<void(ManipId)> ManipCallback;
  typedef boost::shared_ptr<ManipCallback> ManipCallbackPtr;

  static const vector<string> m_arm_joint_names;
	int m_current_joint_ind;

private:
  vector<ManipCallbackPtr> preGrabCallbacks;
	vector<ManipCallbackPtr> preReleaseCallbacks;
	vector<ManipCallbackPtr> postStateCallbacks;

public:
  PR2Object(RaveInstance::Ptr rave);

  void drive(float dx, float dy, float da) { setTransform(getTransform() * btTransform(btQuaternion(0,0,da), btVector3(dx*METERS, dy*METERS, 0))); }
  bool moveByIK(ManipId manip_id, float angle_axis, float finger_normal_axis, float wrist_axis) {
  	return m_manipulators[manip_id]->moveByIK(m_manipulators[manip_id]->getTransform() * btTransform(btQuaternion(0,0,0), btVector3(angle_axis, finger_normal_axis, wrist_axis)*METERS));
  }
  float getGripperAngle(ManipId manip_id) { return m_manipulators[manip_id]->getGripperAngle(); }
  void setGripperAngle(ManipId manip_id, float angle) {
  	m_manipulators[manip_id]->setGripperAngle(angle);
  	update();
  }
  void changeGripperAngle(ManipId manip_id, float delta) {
  	setGripperAngle(manip_id, getGripperAngle(manip_id) + delta);
  }

  float getDOFValue(int index) { return getDOFValues(vector<int>(1,index))[0]; }
  void setDOFValue(int index, float value) { setDOFValues(vector<int>(1,index), vector<double>(1,value)); }
  float getDOFValue(string joint_name) {
  	int ind = robot->GetJointIndex(joint_name);
		if (ind!=-1) return getDOFValue(ind);
		ROS_WARN("Invalid joint name given to getDOFValue");
		return 0.0;
  }
  void setDOFValue(string joint_name, float value) {
  	int ind = robot->GetJointIndex(joint_name);
  	if (ind != -1) {
  		setDOFValue(ind, value);
  		update();
  		return;
  	}
  	ROS_WARN("Invalid joint name given to getDOFValue");
  }
  void changeDOFValue(string joint_name, float delta) {
  	setDOFValue(joint_name, getDOFValue(joint_name)+delta);
  }
  void changeDOFValue(ManipId manip_id, string arm_joint_name, float delta) {
  	string joint_name;
  	if (manip_id == LEFT) joint_name = "l_" + arm_joint_name;
  	else if (manip_id == RIGHT) joint_name = "r_" + arm_joint_name;
  	else { ROS_WARN("Invalid manip_id given to changeDOFValue"); return; }
  	changeDOFValue(joint_name, delta);
  }
  void changeCurrentDOFValue(ManipId manip_id, float delta) {
  	changeDOFValue(manip_id, m_arm_joint_names[m_current_joint_ind], delta);
  }

  void grab(ManipId manip_id = ALL, bool call_callbacks=true);
  void release(ManipId manip_id = ALL, bool call_callbacks=true);
  void setJointState(const sensor_msgs::JointState& msg); // If the grippers are closing and if the target softbody is nearby, then the pr2 grabs the softbody. Gets ignorerd during grab or release.
  string associatedLinkName(const string& joint_name);

	void addPreGrabCallback(ManipCallbackPtr cb) { preGrabCallbacks.push_back(cb); }
	void addPreReleaseCallback(ManipCallbackPtr cb) { preReleaseCallbacks.push_back(cb); }
	void addPostStateCallback(ManipCallbackPtr cb) { postStateCallbacks.push_back(cb); }
	void removePreGrabCallback(ManipCallbackPtr cb) { remove(preGrabCallbacks, cb);	}
	void removePreReleaseCallback(ManipCallbackPtr cb) { remove(preReleaseCallbacks, cb); }
	void removePostStateCallback(ManipCallbackPtr cb) { remove(postStateCallbacks, cb); }

private:
};

bool inline operator==(PR2Object::ManipId lhs, PR2Object::ManipId rhs) {
	return ((int) lhs) == ((int) rhs);
}

#endif /* PR2OBJECT_H_ */



