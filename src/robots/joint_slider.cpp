#include "joint_slider.h"
#include <opencv2/highgui/highgui.hpp>
#include "simulation/openravesupport.h"
#include "robots/robots_fwd.h"
using boost::shared_ptr;
void sliderChanged(int, void* slider) {
	static_cast<JointSlider*>(slider)->m_needsUpdate = true;
}

JointSliderPtr createJointSlider(const std::string manipName, RaveRobotObjectPtr rro) {
	vector<RobotBase::ManipulatorPtr> manips = rro->robot->GetManipulators();
	RobotBase::ManipulatorPtr namedManip;
	BOOST_FOREACH(RobotBase::ManipulatorPtr& manip, manips) if (manip->GetName() == manipName) namedManip = manip;
	return JointSliderPtr(new JointSlider(rro, manipName, namedManip->GetArmIndices()));
}

JointSlider::JointSlider(RaveRobotObjectPtr rro, const std::string& name, const vector<int>& dofInds) :
m_windowName(name), m_rro(rro), m_robot(rro->robot), m_needsUpdate(false), m_dofInds(dofInds) {
  cv::namedWindow(m_windowName);
  vector<KinBody::JointPtr> joints = m_robot->GetJoints();
  m_intVals.resize(joints.size());
  m_robot->GetDOFLimits(m_lower, m_upper);

  vector<double> vals;
  m_robot->GetDOFValues(vals);
  BOOST_FOREACH(int i, m_dofInds) {
  	m_intVals[i] = 255*(vals[i] - m_lower[i])/(m_upper[i]-m_lower[i]);
  	cv::createTrackbar(joints[i]->GetName(), m_windowName, &m_intVals[i], 255, &sliderChanged, this);
  }
}

void JointSlider::updateIfNeeded() {
	cv::waitKey(1);
	if (m_needsUpdate) updateRobot();
	m_needsUpdate = false;
}

void JointSlider::updateRobot() {
	vector<double> vals;
	m_robot->GetDOFValues(vals);
	BOOST_FOREACH(int i, m_dofInds) vals[i] = (m_lower[i]*(255-m_intVals[i]) + m_upper[i]*m_intVals[i])/255.;
	m_robot->SetDOFValues(vals);
	m_rro->updateBullet();
}

JointSlider::~JointSlider() {
	cv::destroyWindow(m_windowName);
}
