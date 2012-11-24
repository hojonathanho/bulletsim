#pragma once
#include <sensor_msgs/JointState.h>
#include <boost/function.hpp>
#include "clouds/ros_robot.h"
#include "robots/grabbing.h"

typedef boost::function<void(void)> VoidCallback;

class GrabDetector {
public:
  typedef boost::shared_ptr<GrabDetector> Ptr;
	enum Side {LEFT, RIGHT};

	// detects grasping by looking for a point when gripper isn't closing
	// but effort is high
	Side m_side;
	int m_grabCount;
	int m_emptyCount;
	static const int toggleCount = 10; // 10 messages / 100 hz = .1 sec
	bool m_grabbing;
	VoidCallback m_grabCB;
	VoidCallback m_releaseCB;
	int m_jointIdx;


	GrabDetector(Side side, VoidCallback grabCB, VoidCallback releaseCB);

	void update(const sensor_msgs::JointState& joint);
	bool isGrabbing(float position, float velocity, float effort);

};

class GrabManager {
public:
  typedef boost::shared_ptr<GrabManager> Ptr;
  Environment::Ptr m_env;
  Monitor::Ptr m_monitor;
  GrabDetector::Ptr m_detector;
  RobotSync* m_sync;
  GrabManager();
  GrabManager(Environment::Ptr env, RaveRobotObject::Manipulator::Ptr arm, GrabDetector::Side, RobotSync* robotSync);
  GrabManager(Environment::Ptr env, RaveRobotObject::Ptr robot, RaveRobotObject::Manipulator::Ptr arm, GrabDetector::Side, RobotSync* robotSync, BulletSoftObject::Ptr target);
  void update();
};
