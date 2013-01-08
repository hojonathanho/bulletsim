#include "lfd_rope_common.h"

#include "utils/logging.h"
#include "utils_python.h"
#include "lfd_python_wrapper.h"

namespace lfd {

//static const float table_dist_from_robot = 0.15;
static const float table_dist_from_robot = 0.2;
static const float table_width = 1, table_length = 1;
static const float table_height = 0.76;
static vector<btVector3> initTableCornersWorld() {
  vector<btVector3> v;
  v.push_back(METERS * btVector3(table_dist_from_robot, -table_width/2, table_height));
  v.push_back(METERS * btVector3(table_dist_from_robot, table_width/2, table_height));
  v.push_back(METERS * btVector3(table_dist_from_robot + table_length, table_width/2, table_height));
  v.push_back(METERS * btVector3(table_dist_from_robot + table_length, -table_width/2, table_height));
  return v;
}

LFDRopeScene::LFDRopeScene(int argc, char *argv[], Config config) {
  GeneralConfig::scale = 10.;
  BulletConfig::maxSubSteps = 100;
  BulletConfig::internalTimeStep = 0.001;
  BulletConfig::gravity = btVector3(0, 0, -1);

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SceneConfig());
  parser.addGroup(config);
  parser.read(argc, argv);

  LoggingInit();
  Python_setup();
}

void LFDRopeScene::resetScene(const vector<btVector3> &ropeCtlPts) {
  vector<btVector3> tableCorners = initTableCornersWorld();

  scene.reset(new TableRopeScene(tableCorners, ropeCtlPts));
  scene->m_table->rigidBody->setFriction(0.9);
  scene->m_table->rigidBody->getCollisionShape()->setMargin(0.1);
  scene->m_table->setColor(0, 1, 0, 0.2);

  scene->stepFor(DT, 1);
}

RopeState ropeState_sim2real(const RopeState &rs, RaveRobotObject::Ptr pr2) {
  btTransform base_footprint_trans = pr2->getLinkTransform(
    pr2->robot->GetLink("base_footprint")
  );
  RopeState rs2;
  for (int i = 0; i < rs.size(); ++i) {
    rs2.push_back(base_footprint_trans.inverse() * rs[i] / METERS);
  }
  return rs2;
}

RopeState ropeState_real2sim(const RopeState &rs, RaveRobotObject::Ptr pr2) {
  btTransform base_footprint_trans = pr2->getLinkTransform(
    pr2->robot->GetLink("base_footprint")
  );
  RopeState rs2;
  for (int i = 0; i < rs.size(); ++i) {
    rs2.push_back(base_footprint_trans * rs[i] * METERS);
  }
  return rs2;
}

void RopeStatePlot::setRope(const RopeState &rs, const Eigen::Vector3f &color, float alpha) {
	vector<btVector3> linePoints;
	vector<btVector4> lineColors;
	for (int i=0; i<rs.size()-1; i++) {
		linePoints.push_back(rs[i]);
		linePoints.push_back(rs[i+1]);
		lineColors.push_back(btVector4(color(0), color(1), color(2), alpha));
	}
  setPoints(linePoints, lineColors);
}

RopeState loadRopeStateFromDemoCloud(const string &demo_task, const string &demo_seg) {
  // load rope from h5 file
  DemoLoadingModule demoLoader;
  py::object demos = demoLoader.loadDemos(demo_task);
  RopeInitModule ropeInitializer;
  py::object pyrope = ropeInitializer.find_path_through_point_cloud(demos[demo_seg]["cloud_xyz"]);
  vector<btVector3> ropeCtlPts;
  for (int i = 0; i < py::len(pyrope); ++i) {
    btScalar x = py::extract<btScalar>(pyrope[i][0]);
    btScalar y = py::extract<btScalar>(pyrope[i][1]);
    btScalar z = py::extract<btScalar>(pyrope[i][2]);
    ropeCtlPts.push_back(btVector3(x, y, z) * METERS);
  }
  return ropeCtlPts;
}


} // namespace lfd
