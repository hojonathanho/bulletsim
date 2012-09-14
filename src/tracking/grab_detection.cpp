#include <cmath>
#include "grab_detection.h"
#include <algorithm>
#include <cstdio>
#include <string>

GrabDetector::GrabDetector(Side side, VoidCallback grabCB, VoidCallback releaseCB) :
	m_side(side),
	m_grabCount(0),
	m_emptyCount(0),
	m_grabbing(false),
	m_grabCB(grabCB),
	m_releaseCB(releaseCB),
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
	bool out = position < .04;
	return out;
}

GrabManager::GrabManager(Environment::Ptr env, RaveRobotObject::Manipulator::Ptr manip, GrabDetector::Side side, RobotSync& robotSync) :
    m_env(env),
    m_monitor(manip, env->bullet->dynamicsWorld),
    m_detector(side, boost::bind(&MonitorForGrabbing::grab, &m_monitor), boost::bind(&MonitorForGrabbing::release, &m_monitor)),
    m_sync(robotSync)
{}

void GrabManager::update() {
  m_detector.update(m_sync.m_lastMsg);
  m_monitor.updateGrabPose();
}
