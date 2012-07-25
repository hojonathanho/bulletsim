#include "hydra_pr2_teleop.h"
#include "simulation/simplescene.h"
#include "utils/conversions.h"
#include <ros/ros.h>
#include "simulation/bullet_io.h"
#include "utils/logging.h"
const static int START = 0;
const static int BUMPER = 5;
const static int JOYSTICK = 6;

HydraPR2Teleop::HydraPR2Teleop(RaveRobotObject::Ptr pr2, Scene* scene)
: m_pr2(pr2),
  m_left(m_pr2->createManipulator("leftarm")),
  m_right(m_pr2->createManipulator("rightarm")),
  m_scene(scene)
{
	if (!ros::isInitialized()) {
		int argc = 0;
		char** argv = {};
		ros::init(argc, argv,"teleoperator",ros::init_options::AnonymousName);
	}
	m_nhPtr = ros::NodeHandlePtr(new ros::NodeHandle());
	m_sub = ros::Subscriber(m_nhPtr->subscribe("/hydra_calib", 1, &HydraPR2Teleop::callback, this));
	m_leftAx.reset(new PlotAxes());
	m_rightAx.reset(new PlotAxes());
	scene->env->add(m_leftAx);
	scene->env->add(m_rightAx);
	scene->addPreStepCallback(&ros::spinOnce);
}

void HydraPR2Teleop::callback(const hydra_msgs::Calib& msg) {
	static btMatrix3x3 eeRot(0,0,-1,0,1,0,1,0,0);
	static btVector3 offsets[2] = {btVector3(0,-1*METERS,0), btVector3(0,1*METERS,0)};
	static RaveRobotObject::Manipulator::Ptr manips[2] = {m_left, m_right};
	static PlotAxes::Ptr plots[2] = {m_leftAx, m_rightAx};
	for (int i_manip = 0; i_manip < 2; i_manip++) {
		btTransform trans = toBulletTransform(msg.paddles[i_manip].transform);
		btVector3 orig = trans.getOrigin() / 200 + btVector3(3,0,.5)*METERS + offsets[i_manip];
//		orig.setY(-orig.y());
//		orig.setX(orig.x());
		trans.setOrigin(orig);
		plots[i_manip]->setup(trans,.1*METERS);
//		btMatrix3x3 rot = btMatrix3x3(-1,0,0,0,-1,0,0,0,1);
		trans.setBasis(trans.getBasis()*eeRot.transpose());
//		trans.setBasis(eeRot*trans.getBasis());
		bool engaged = msg.paddles[i_manip].buttons[BUMPER];
		if (engaged) manips[i_manip]->moveByIK(trans, false, false);
		LOG_DEBUG(trans);
	}
}
