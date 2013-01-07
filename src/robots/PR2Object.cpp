/*
 * PR2Object.cpp
 *
 *  Created on: Nov 16, 2012
 *      Author: alex
 */

#include "PR2Object.h"
#include "robots/ros2rave.h"
#include <boost/assign.hpp>

const vector<PR2Object::ManipId> PR2Object::m_manip_ids = boost::assign::list_of(LEFT)(RIGHT);
const vector<string> PR2Object::m_arm_joint_names = boost::assign::list_of("shoulder_pan_joint")("shoulder_lift_joint")("upper_arm_roll_joint")
		("elbow_flex_joint")("forearm_roll_joint")("wrist_flex_joint")("wrist_roll_joint");

PR2Object::PR2Object(RaveInstance::Ptr rave)
	: RaveRobotObject(rave, "robots/pr2-beta-static.zae")
	, m_current_joint_ind(0)
	, m_disable_set_joint_state(false)
{}

PR2Object::PR2Object()
	: m_current_joint_ind(0)
	, m_disable_set_joint_state(false)
{}

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
		m_detectors[manip_id].reset(new HysterisGrabDetector(0.02, 0.03,
				boost::bind(&PR2Object::grab, shared_from_this(), manip_id, true),
				boost::bind(&PR2Object::release, shared_from_this(), manip_id, true)));
	}

	// Environment callbacks (they get called by the Scene owning this environment (if any))
	// Environment callbacks for moving the base
	getEnvironment()->addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Left, boost::bind(&PR2Object::drive, this, 0,.05, 0), "(left arrow) Translate the PR2 to the left");
	getEnvironment()->addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Right, boost::bind(&PR2Object::drive, this, 0,-.05, 0), "(right arrow) Translate the PR2 to the right");
	getEnvironment()->addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Up, boost::bind(&PR2Object::drive, this, .05,0,  0), "(up arrow) Translate the PR2 forward");
	getEnvironment()->addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Down, boost::bind(&PR2Object::drive, this, -.05,0, 0), "(down arrow) Translate the PR2 backward");
	getEnvironment()->addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Leftbracket, boost::bind(&PR2Object::drive, this, 0,0, .05), "Rotate the PR2 to the left");
	getEnvironment()->addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Rightbracket, boost::bind(&PR2Object::drive, this, 0,0, -.05), "Rotate the PR2 to the right");

//	getEnvironment()->addVoidKeyCallback('u',boost::bind(&PR2Object::moveByIK, this, LEFT, 0.01,0,0), "move left gripper along angle axis +");
//  getEnvironment()->addVoidKeyCallback('o',boost::bind(&PR2Object::moveByIK, this, LEFT, -0.01,0,0), "move left gripper along angle axis -");
//  getEnvironment()->addVoidKeyCallback('j',boost::bind(&PR2Object::moveByIK, this, LEFT, 0,0.01,0), "move left gripper along finger normal axis +");
//  getEnvironment()->addVoidKeyCallback('l',boost::bind(&PR2Object::moveByIK, this, LEFT, 0,-0.01,0), "move left gripper along finger normal axis -");
//	getEnvironment()->addVoidKeyCallback('i',boost::bind(&PR2Object::moveByIK, this, LEFT, 0,0,0.01), "move left gripper along wrist axis +");
//  getEnvironment()->addVoidKeyCallback('k',boost::bind(&PR2Object::moveByIK, this, LEFT, 0,0,-0.01), "move left gripper along wrist axis -");
//
//	// Environment callbacks for changing gripper angles
//  getEnvironment()->addVoidKeyCallback('x',boost::bind(&PR2Object::changeGripperAngle, this, LEFT, 0.01), "increase the left gripper's angle");
//  getEnvironment()->addVoidKeyCallback('z',boost::bind(&PR2Object::changeGripperAngle, this, LEFT, -0.01), "decrease the left gripper's angle");
//  getEnvironment()->addVoidKeyCallback('X',boost::bind(&PR2Object::changeGripperAngle, this, RIGHT, 0.01), "increase the right gripper's angle");
//  getEnvironment()->addVoidKeyCallback('Z',boost::bind(&PR2Object::changeGripperAngle, this, RIGHT, -0.01), "decrease the right gripper's angle");
//
//  // Environment callbacks for changing arm DOF's
//	getEnvironment()->addVoidKeyCallback('m',boost::bind(add, &m_current_joint_ind, 1, m_arm_joint_names.size()), "change the current DOF ind to the next one");
//	getEnvironment()->addVoidKeyCallback('n',boost::bind(add, &m_current_joint_ind, -1, m_arm_joint_names.size()), "change the current DOF ind to the previous one");
//	getEnvironment()->addVoidKeyCallback('b',boost::bind(&PR2Object::changeCurrentDOFValue, this, LEFT, 0.01), "change current left arm DOF value +");
//	getEnvironment()->addVoidKeyCallback('v',boost::bind(&PR2Object::changeCurrentDOFValue, this, LEFT, -0.01), "change current left arm DOF value -");
//	getEnvironment()->addVoidKeyCallback('B',boost::bind(&PR2Object::changeCurrentDOFValue, this, RIGHT, 0.01), "change current right arm DOF value +");
//	getEnvironment()->addVoidKeyCallback('V',boost::bind(&PR2Object::changeCurrentDOFValue, this, RIGHT, -0.01), "change current right arm DOF value -");
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

void PR2Object::grab(ManipId manip_id, bool call_callbacks) {
	if (manip_id == ALL) {
		BOOST_FOREACH(ManipId manip_id, m_manip_ids) grab(manip_id);
	} else {
		m_disable_set_joint_state = true;
		if (call_callbacks) BOOST_FOREACH(ManipCallbackPtr& preGrabCallback, preGrabCallbacks) (*preGrabCallback)(manip_id);
		m_monitors[manip_id]->grab();
		cout << "PR2Object::grab" << endl;
		m_disable_set_joint_state = false;
	}
}

void PR2Object::release(ManipId manip_id, bool call_callbacks) {
	if (manip_id == ALL) {
		BOOST_FOREACH(ManipId manip_id, m_manip_ids) release(manip_id);
	} else {
		m_disable_set_joint_state = true;
		if (call_callbacks) BOOST_FOREACH(ManipCallbackPtr& preReleaseCallback, preReleaseCallbacks) (*preReleaseCallback)(manip_id);
		m_monitors[manip_id]->release();
		cout << "PR2Object::release" << endl;
		m_disable_set_joint_state = false;
	}
}

void PR2Object::setJointState(const sensor_msgs::JointState& msg) {
  if (m_disable_set_joint_state) return;

	setupROSRave(robot, msg);
  ValuesInds vi = getValuesInds(msg.position);
  setDOFValues(vi.second, vi.first);

  update();
}

string PR2Object::associatedLinkName(const string& joint_name) {
	string link_name = joint_name.substr(0, joint_name.size()-6);
	if (link_name == "r_gripper") link_name = "r_gripper_palm";
	else if (link_name == "l_gripper") link_name = "l_gripper_palm";
	else runtime_error("Unknown joint name");
	link_name = link_name + "_link";
	return link_name;
}

void PR2Object::update() {
	BOOST_FOREACH(ManipId manip_id, m_manip_ids) {
		m_detectors[manip_id]->update(m_manipulators[manip_id]->getGripperAngle());
		m_monitors[manip_id]->updateGrabPose();
		BOOST_FOREACH(ManipCallbackPtr& postStateCallback, postStateCallbacks)
			(*postStateCallback)(manip_id);
	}
}
