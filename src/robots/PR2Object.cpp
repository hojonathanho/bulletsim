/*
 * PR2Object.cpp
 *
 *  Created on: Nov 16, 2012
 *      Author: alex
 */

#include "PR2Object.h"
#include "robots/ros2rave.h"

PR2Object::PR2Object(RaveInstance::Ptr rave) : RaveRobotObject(rave, "robots/pr2-beta-static.zae") {
	m_manip_ids.push_back(LEFT);
	m_manip_ids.push_back(RIGHT);
	assert(m_manip_ids.size() == MANIP_ID_SIZE);
}

PR2Object::PR2Object() {
	m_manip_ids.push_back(LEFT);
	m_manip_ids.push_back(RIGHT);
	assert(m_manip_ids.size() == MANIP_ID_SIZE);
}

void PR2Object::init() {
	//other options are not supported
	assert(!SceneConfig::useFakeGrabber && SceneConfig::enableRobot && SceneConfig::enableIK);

	RaveRobotObject::init();

	assert((numCreatedManips() == 0) || (numCreatedManips() == MANIP_ID_SIZE)); // the base class either has no manip (when it's just created) or MANIP_IP_SIZE (when it's forked)
	m_manipulators.resize(MANIP_ID_SIZE);
	if (numCreatedManips() == 0) {
		m_manipulators[LEFT]  = createManipulator("leftarm", false);
		m_manipulators[RIGHT] = createManipulator("rightarm", false);
	} else {
		m_manipulators[LEFT] = getManipByName("leftarm");
		assert(m_manipulators[LEFT]);
		m_manipulators[RIGHT] = getManipByName("rightarm");
		assert(m_manipulators[RIGHT]);
	}

	m_monitors.resize(MANIP_ID_SIZE);
	m_detectors.resize(MANIP_ID_SIZE);
	BOOST_FOREACH(ManipId manip_id, m_manip_ids) {
		SoftMonitorForGrabbing::Ptr smfg(new SoftMonitorForGrabbing(shared_from_this(), m_manipulators[manip_id], manip_id == LEFT));
		smfg->gripper->setGrabOnlyOnContact(true);
		m_monitors[manip_id] = smfg;
		m_detectors[manip_id].reset(new GrabDetector(manip_id == LEFT ? GrabDetector::LEFT : GrabDetector::RIGHT,
				boost::bind(&SoftMonitorForGrabbing::grab, smfg.get()),
				boost::bind(&SoftMonitorForGrabbing::release, smfg.get())));
	}

	// Environment callbacks (they get called by the Scene owning this environment (if any))
	getEnvironment()->addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Left, boost::bind(&PR2Object::drive, this, 0,.05, 0), "(left arrow) Translate the PR2 to the left");
	getEnvironment()->addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Right, boost::bind(&PR2Object::drive, this, 0,-.05, 0), "(right arrow) Translate the PR2 to the right");
	getEnvironment()->addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Up, boost::bind(&PR2Object::drive, this, .05,0,  0), "(up arrow) Translate the PR2 forward");
	getEnvironment()->addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Down, boost::bind(&PR2Object::drive, this, -.05,0, 0), "(down arrow) Translate the PR2 backward");
	getEnvironment()->addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Leftbracket, boost::bind(&PR2Object::drive, this, 0,0, .05), "Rotate the PR2 to the left");
	getEnvironment()->addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Rightbracket, boost::bind(&PR2Object::drive, this, 0,0, -.05), "Rotate the PR2 to the right");
}

EnvironmentObject::Ptr PR2Object::copy(Fork &f) const {
	Ptr o(new PR2Object());
  RaveRobotObject::internalCopy(o, f);
	return o;
}

void PR2Object::postCopy(EnvironmentObject::Ptr copy, Fork &f) const {
	Ptr o = boost::static_pointer_cast<PR2Object>(copy);
	BOOST_FOREACH(ManipId manip_id, m_manip_ids) {
		PR2SoftBodyGripper::Ptr gripper = boost::shared_dynamic_cast<SoftMonitorForGrabbing>(m_monitors[manip_id])->gripper;
		PR2SoftBodyGripper::Ptr o_gripper = boost::shared_dynamic_cast<SoftMonitorForGrabbing>(o->m_monitors[manip_id])->gripper;

		o_gripper->grabOnlyOnContact = gripper->grabOnlyOnContact;
		for (int i=0; i<gripper->anchors.size(); i++) {
			BulletSoftObject::Ptr o_bso = boost::static_pointer_cast<BulletSoftObject>(f.forkOf(gripper->anchors[i].m_bso));
			o_gripper->anchors.push_back(PR2SoftBodyGripper::Anchor(o_bso, gripper->anchors[i].m_idx));
		}
	}
}

void PR2Object::grab(ManipId manip_id) {
	if (manip_id == ALL)
		BOOST_FOREACH(Monitor::Ptr& monitor, m_monitors) monitor->grab();
	else
		m_monitors[manip_id]->grab();
}

void PR2Object::release(ManipId manip_id) {
	if (manip_id == ALL)
		BOOST_FOREACH(Monitor::Ptr& monitor, m_monitors) monitor->release();
	else
		m_monitors[manip_id]->release();
}

void PR2Object::setJointState(const sensor_msgs::JointState& msg) {
  setupROSRave(robot, msg);
  ValuesInds vi = getValuesInds(msg.position);
  setDOFValues(vi.second, vi.first);

  BOOST_FOREACH(GrabDetector::Ptr& detector, m_detectors)
  	detector->update(msg);
  BOOST_FOREACH(Monitor::Ptr& monitor, m_monitors)
    monitor->updateGrabPose();
}

