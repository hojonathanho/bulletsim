#include "simulation/openravesupport.h"
#include "simulation/plotting.h"
#include "hydra_msgs/Calib.h"
#include <ros/subscriber.h>

class Scene;
class HydraPR2Teleop {
public:
	RaveRobotObject::Ptr m_pr2;
	RaveRobotObject::Manipulator::Ptr m_left, m_right;
	ros::NodeHandlePtr m_nhPtr;
	ros::Subscriber m_sub;
	Scene* m_scene;
	PlotAxes::Ptr m_leftAx, m_rightAx;
	HydraPR2Teleop(RaveRobotObject::Ptr, Scene*);
private:
	void callback(const hydra_msgs::Calib&);
};
