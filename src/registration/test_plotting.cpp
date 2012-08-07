#include "plotting.h"
#include "clouds/utils_pcl.h"
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
int main(int argc, char* argv[]) {
	ColorCloudPtr cups = readPCD("/home/joschu/Data/cups/cups1.pcd");
	PlotPoints::Ptr plot_points(new PlotPoints(2));
	plot_points->setPoints(cups);

	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->addChild(plot_points.get());
	osg::ref_ptr<osgGA::TrackballManipulator> manip =
			new osgGA::TrackballManipulator();

	osgViewer::Viewer viewer;
	manip->setHomePosition(osg::Vec3f(10, 0, 0), osg::Vec3f(11, 1, 0),
			osg::Vec3f(10, 0, 1));
	osg::ref_ptr<osg::Camera> cam = viewer.getCamera();
	viewer.setCameraManipulator(manip);
	viewer.setUpViewInWindow(0, 0, 1000, 1000);
	viewer.realize();
	viewer.setSceneData(root.get());

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	root->addChild(geode.get());

	for (int i = 0; i < 10; i++) {
		osg::Sphere* sphere = new osg::Sphere(osg::Vec3f(i + 1, 10, 0), .1 * i);
		osg::ShapeDrawable* sphereDrawable = new osg::ShapeDrawable(sphere);
		geode->addDrawable(sphereDrawable);
	}

	while (true) {
		viewer.frame();
		sleep(.01);
	}

}
