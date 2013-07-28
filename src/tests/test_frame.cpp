#include "simulation/simplescene.h"
#include "simulation/config_viewer.h"
#include "simulation/recording.h"
#include "robots/ravens.h"
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/WriteFile>

using namespace osg;

struct SaveImageOp: public osgViewer::ScreenCaptureHandler::CaptureOperation {
	string fname;

	SaveImageOp(string _fname) : osgViewer::ScreenCaptureHandler::CaptureOperation() {
		cout << "constructed" <<endl;
		fname = _fname;
	}

	void operator () (const osg::Image& image, const unsigned int context_id) {
		cout << " CALLED"<<endl;
		osgDB::writeImageFile(image, fname);
	}
};


class CaptureScreen {
public:
	osg::ref_ptr<osgViewer::ScreenCaptureHandler> m_captureHandler;
	osgViewer::Viewer& m_viewer;
	SaveImageOp* m_capture_op;

	CaptureScreen(osgViewer::Viewer& viewer, string fname) : m_viewer(viewer) {
		m_capture_op = new SaveImageOp(fname);
		m_captureHandler = new osgViewer::ScreenCaptureHandler(m_capture_op);
	}

	void snapshot() {
		cout << "calling ..." <<endl;
		m_captureHandler->captureNextFrame(m_viewer);
		m_captureHandler->setFramesToCapture(1);
		cout << m_capture_op->fname <<endl;
	}
};


class CustomScene : public Scene {
public:
	BulletInstance::Ptr bullet2;
	OSGInstance::Ptr osg2;
	Fork::Ptr fork;
	RaveRobotObject::Ptr origRobot, tmpRobot;
	Ravens ravens;

	PlotPoints::Ptr plot_points;
	PlotAxes::Ptr plot_axes1;
	PlotAxes::Ptr plot_axes2;

	void callGripperAction(char lr) {}

	CustomScene(): ravens(*this){}

	void setup() {
		//BulletConfig::internalTimeStep = 0.001;
		const float dt = BulletConfig::dt;

		// position the ravens
		btTransform T;
		T.setIdentity();
		T.setOrigin(btVector3(0,0,0.05));
		ravens.applyTransform(util::toRaveTransform(T));

		// set up the points for plotting
		plot_points.reset(new PlotPoints(5));
		env->add(plot_points);
		plot_axes1.reset(new PlotAxes());
		env->add(plot_axes1);
		plot_axes2.reset(new PlotAxes());
		env->add(plot_axes2);

		ravens.setArmPose("home",'b');

		//startViewer();
		stepFor(dt, 1.);
	}
};



int main(int argc, char *argv[]) {
	// first read the configuration from the user

	// and override config values to what we want
	//SceneConfig::enableIK = true;
	//SceneConfig::enableRobot = true;

	// construct the scene
	CustomScene scene;
	scene.setup();
	// manipulate the scene or add more objects, if desired
	// start the simulation
	//scene.startViewer();
	//scene.step(3);

	ViewerConfig::windowHeight = 768;
	ViewerConfig::windowWidth = 1024;
	ViewerConfig::cameraHomePosition = btVector3(0, 0.45, 0.5);
	ViewerConfig::cameraHomeCenter = btVector3(0, 0, 0.15);

	scene.viewer.setUpViewInWindow(0, 0, ViewerConfig::windowWidth, ViewerConfig::windowHeight);
	osg::ref_ptr<EventHandler>  manip = new EventHandler(scene);
	manip->setHomePosition(util::toOSGVector(ViewerConfig::cameraHomePosition)*METERS, util::toOSGVector(ViewerConfig::cameraHomeCenter)*METERS, util::toOSGVector(ViewerConfig::cameraHomeUp)*METERS);
	manip->setWheelZoomFactor(ViewerConfig::zoomFactor);
	scene.viewer.setCameraManipulator(manip);
	scene.viewer.setSceneData(scene.osg->root.get());

	osg::Light* light1 = new osg::Light();
	osg::LightSource * lightsource1 = new osg::LightSource();
	lightsource1->setLight(light1);
	scene.osg->root->addChild(lightsource1);
	osg::StateSet* stateset1 = scene.osg->root->getOrCreateStateSet();
	lightsource1->setStateSetModes(*stateset1, osg::StateAttribute::ON);
	light1->setAmbient(osg::Vec4d(0.2, 0.2, 0.2, 1.0));
	light1->setDiffuse(osg::Vec4d(1.0, 1.0, 1.0, 1.0));
	light1->setSpecular(osg::Vec4d(0.5, 0.5, 0.5, 1.0));
	light1->setPosition(osg::Vec4d(0*METERS, 0.5*METERS, METERS*.50, 1.0));
	//scene.viewer.realize();

	//ScreenCapture(scene.viewer).snapshot();
	CaptureScreen(scene.viewer, "/home/ankush/test_image1.jpg").snapshot();
	scene.viewer.frame();
	return 0;
}
