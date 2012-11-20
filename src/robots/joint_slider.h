#pragma once
#include "simulation/simulation_fwd.h"
#include "simulation/openrave_fwd.h"
#include "robots/robots_fwd.h"
#include <vector>
#include <string>
using std::vector;
using std::string;

struct JointSlider {
	vector<int> m_intVals;
	vector<double> m_lower, m_upper;
	string m_windowName;
	bool m_needsUpdate;
	RaveRobotObjectPtr m_rro;
	OpenRAVE::RobotBasePtr m_robot;
	vector<int> m_dofInds;
	JointSlider(RaveRobotObjectPtr, const string& name, const vector<int>& dofInds);
	~JointSlider();
	void updateRobot();
	void updateIfNeeded();
};

JointSliderPtr createJointSlider(const std::string manipName, RaveRobotObjectPtr rro);
