#include <cmath>
#include "grab_detection.h"
#include <algorithm>
#include <cstdio>
#include <string>

GrabDetector::GrabDetector(Side side, VoidCallback grabCB, VoidCallback releaseCB) :
	GrabDetectorBase(grabCB, releaseCB),
	m_side(side),
	m_grabCount(0),
	m_emptyCount(0),
	m_jointIdx(-1) {}


void GrabDetector::update(const sensor_msgs::JointState& joint) {
	if (m_jointIdx == -1) {
		std::string jointName = (m_side == LEFT) ? "l_gripper_joint" : "r_gripper_joint";
		m_jointIdx = std::find(joint.name.begin(), joint.name.end(), jointName) - joint.name.begin();
		assert(m_jointIdx != joint.name.size());
		printf("joint index: %i\n",m_jointIdx);
	}
	float position = joint.position[m_jointIdx];
	float velocity = joint.velocity[m_jointIdx];
	float effort = joint.effort[m_jointIdx];
	bool grabNow = isGrabbing(position, velocity, effort);
	if (grabNow) {
		m_emptyCount = 0;
		m_grabCount++;
		if (m_grabCount == toggleCount) {
			if (!m_grabbing) {
				m_grabbing = true;
				printf("no grab -> grab\n");
				m_grabCB();
			}
		}
	}
	else {
		m_grabCount = 0;
		m_emptyCount++;
		if (m_emptyCount == toggleCount) {
			if (m_grabbing) {
				m_grabbing = false;
				printf("grab -> no grab\n");
				m_releaseCB();
			}
		}
	}
}

bool GrabDetector::isGrabbing(float position, float velocity, float effort) {
//	bool out= (fabs(velocity) < .001) && (fabs(effort) > 50) && (position > 0);
	bool out = position < .01;
	return out;
}

HysterisGrabDetector::HysterisGrabDetector(float gripper_angle_low_thresh, float gripper_angle_high_thresh, VoidCallback grabCB, VoidCallback releaseCB) :
	GrabDetectorBase(grabCB, releaseCB),
	m_high_thresh(gripper_angle_high_thresh),
	m_low_thresh(gripper_angle_low_thresh)
{
	if (m_low_thresh > m_high_thresh) runtime_error("The high threshold for the gripper angle should be greater than the low threshold");
}

void HysterisGrabDetector::update(float gripper_angle) {
	if (!m_grabbing && (gripper_angle<m_low_thresh)) {
		m_grabbing = true;
		printf("no grab -> grab\n");
		m_grabCB();
	} else if (m_grabbing && (gripper_angle>m_high_thresh)) {
		m_grabbing = false;
		printf("grab -> no grab\n");
		m_releaseCB();
	}
}

GrabManager::GrabManager() {}

GrabManager::GrabManager(Environment::Ptr env, RaveRobotObject::Manipulator::Ptr manip, GrabDetector::Side side, RobotSync* robotSync) :
    m_env(env),
    m_sync(robotSync)
{
  MonitorForGrabbing::Ptr mfg(new MonitorForGrabbing(manip, env->bullet->dynamicsWorld));
  m_monitor = mfg;
  m_detector.reset(new GrabDetector(side,
      boost::bind(&MonitorForGrabbing::grab, mfg),
      boost::bind(&MonitorForGrabbing::release, mfg)));
}

GrabManager::GrabManager(Environment::Ptr env, RaveRobotObject::Ptr robot, RaveRobotObject::Manipulator::Ptr manip, GrabDetector::Side side, RobotSync* robotSync) :
    m_env(env),
    m_sync(robotSync)
{
  bool leftGripper = side==GrabDetector::LEFT;
  SoftMonitorForGrabbing::Ptr smfg(new SoftMonitorForGrabbing(robot, manip, leftGripper));
  m_monitor = smfg;
  m_detector.reset(new GrabDetector(side,
      boost::bind(&SoftMonitorForGrabbing::grab, smfg.get()),
      boost::bind(&SoftMonitorForGrabbing::release, smfg.get())));
}

void GrabManager::update() {
  m_detector->update(m_sync->m_lastMsg);
  m_monitor->updateGrabPose();
}
