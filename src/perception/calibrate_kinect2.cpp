#include "simulation/bullet_io.h"
#include "clouds/comm_pcl.h"
#include "clouds/geom.h"
#include "clouds/utils_pcl.h"
#include "comm/comm.h"
#include "simulation/config_bullet.h"
#include "config_perception.h"
#include "simulation/simplescene.h"
#include "utils_perception.h"
#include "perception/plotting_perception.h"
#include "utils/vector_io.h"
#include "robot_geometry.h"
#include "robots/ros2rave.h"
#include "robots/pr2.h"
#include "simulation/bullet_io.h"

#include <pcl/common/transforms.h>
#include <osgViewer/ViewerEventHandlers>

static btTransform rotX(btTransform t, float s) {
	return t * btTransform(btQuaternion(s, 0, 0, 1), btVector3(0, 0, 0));
}
static btTransform rotY(btTransform t, float s) {
	return t * btTransform(btQuaternion(0, s, 0, 1), btVector3(0, 0, 0));
}
static btTransform rotZ(btTransform t, float s) {
	return t * btTransform(btQuaternion(0, 0, s, 1), btVector3(0, 0, 0));
}
static btTransform moveX(btTransform t, float s) {
	return t * btTransform(btQuaternion(0, 0, 0, 1), btVector3(s, 0, 0));
}
static btTransform moveY(btTransform t, float s) {
	return t * btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, s, 0));
}
static btTransform moveZ(btTransform t, float s) {
	return t * btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, s));
}

vector<double> getTimes(string topic, string ext) {
	vector<double> times;

	int id = 0;
	while (true) {
		PathPair pair = makePathPair(id, ext, topic);
		if (!exists(pair.first))
			break;
		Json::Value val = readJson(pair.second);
		double t = val["time"].asDouble();
		times.push_back(t);
		id++;
	}

	return times;
}

int linearSearch(double t, const vector<double>& times) {
	int i;
	for (i = 0; i < times.size(); i++) {
		if (times[i] > t)
			break;
	}
	return i;
}


struct CalibKeyHandler: public osgGA::GUIEventHandler {
	btTransform m_transform;
	void setTrans(btTransform trans) {
		m_transform = trans;
	}

	CalibKeyHandler() :
		m_transform(btTransform::getIdentity()) {
	}

	bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &) {

#define KEY_CONTROL_LEFT(key, incfunc, delta)				\
  case key:								\
    setTrans(incfunc(m_transform,delta)); \
    return true
#define KEY_CONTROL_RIGHT(key, incfunc, delta)				\
  case key:								\
    setTrans(incfunc(m_transform,delta)); \
    return true

		switch (ea.getEventType()) {
		case osgGA::GUIEventAdapter::KEYDOWN:
			switch (ea.getKey()) {

			KEY_CONTROL_RIGHT('p',rotX,.01)
				;
			KEY_CONTROL_RIGHT('P',rotX,-.01)
				;
			KEY_CONTROL_RIGHT('o',rotY,.01)
				;
			KEY_CONTROL_RIGHT('O',rotY,-.01)
				;
			KEY_CONTROL_RIGHT('i',rotZ,.01)
				;
			KEY_CONTROL_RIGHT('I',rotZ,-.01)
				;
			KEY_CONTROL_RIGHT('l',moveX,.01)
				;
			KEY_CONTROL_RIGHT('L',moveX,-.01)
				;
			KEY_CONTROL_RIGHT('k',moveY,.01)
				;
			KEY_CONTROL_RIGHT('K',moveY,-.01)
				;
			KEY_CONTROL_RIGHT('j',moveZ,.01)
				;
			KEY_CONTROL_RIGHT('J',moveZ,-.01)
				;
			}
			break;
		}
		return false;

#undef KEY_CONTROL_LEFT
#undef KEY_CONTROL_RIGHT
	}
};

struct CalibScene: public Scene {

	PlotAxes::Ptr axes;
	PointCloudPlot::Ptr kinectPlot;

	vector<double> jointTimes;
	vector<double> cloudTimes;

	CloudMessage cloudMsg;
	VectorMessage<double> jointMsg;

	CalibKeyHandler* kh;
	PR2Manager pr2m;

	KinectTransformer kt;

	btTransform wfc;

	int iCloud;

	void fwdTime() {
		if (iCloud == cloudTimes.size() - 1)
			return;
		iCloud += 1;
		updateRobot();
		updateCloud();
	}
	void backTime() {
		if (iCloud == 0)
			return;
		iCloud -= 1;
		updateRobot();
		updateCloud();
	}

	void updateRobot() {
		int iJoint = linearSearch(cloudTimes[iCloud], jointTimes);
		PathPair pair = makePathPair(iJoint, "txt", "joint_states");
		jointMsg.fromFiles(pair);
		ValuesInds vi = getValuesInds(jointMsg.m_data);
		pr2m.pr2->setDOFValues(vi.second, vi.first);
	}

	void updateCloud() {
		PathPair pair = makePathPair(iCloud, "pcd", "kinect");
		cloudMsg.fromFiles(pair);
		kinectPlot->setPoints1(transformPointCloud1(cloudMsg.m_data,
				toEigenTransform(wfc)));
	}

	void updateTransform() {
		kt.headFromKinect = kh->m_transform;
		wfc = kt.getWFC();
	}

	void step() {
		updateRobot();
		updateTransform();
		updateCloud();
		axes->setup(wfc, .25);
		Scene::step(0);
	}

	CalibScene() :
		Scene(), iCloud(0), axes(new PlotAxes()), kinectPlot(new PointCloudPlot()), pr2m(*this), kt(pr2m.pr2->robot),
				jointTimes(getTimes("joint_states", "txt")), cloudTimes(
						getTimes("kinect", "pcd")) {
		addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Left, boost::bind(
				&CalibScene::backTime, this));
		addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Right, boost::bind(
				&CalibScene::fwdTime, this));
		kh = new CalibKeyHandler();
		viewer.addEventHandler(kh);


		BOOST_FOREACH(BulletObject::Ptr child, pr2m.pr2->children) {
			if (child) child->setColor(1,0,0,.3);
		}

		env->add(axes);
		env->add(kinectPlot);
	}

};

struct LocalConfig : public Config {
  static string input;
  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("input", &input, "input transform files"));
  }
};
string LocalConfig::input = "";

extern btTransform transformFromFile(const string& name);

int main(int argc, char* argv[]) {
	//////////// get command line options
	Parser parser;
	SceneConfig::enableIK = SceneConfig::enableHaptics = false;
	SceneConfig::enableRobot = true;
	SceneConfig::enableRobotCollision = false;
	GeneralConfig::scale = 1;
	parser.addGroup(SceneConfig());
	parser.addGroup(GeneralConfig());
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	initComm();
	CalibScene scene;

	if (LocalConfig::input.size() > 0) {
	  btTransform trans = transformFromFile(LocalConfig::input);
	  scene.kh->m_transform = trans;
	}

	scene.startViewer();
	while (!scene.viewer.done())
		scene.step();
	cout << "saving transform to transform.txt" << endl;
	cout << "(maps kinect coords to wide_stereo_gazebo_l_stereo_camera_optical_frame)" << endl;
	ofstream outfile(onceFile("transform.txt").string().c_str());
	outfile << scene.kt.headFromKinect << endl;
	outfile.close();
	return 0;
}
