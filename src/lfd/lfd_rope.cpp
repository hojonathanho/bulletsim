#include "task_execution.h"
#include "rope_scenes.h"

#include "utils/logging.h"
#include "utils/vector_io.h"
#include "utils/conversions.h"

#include "lfd_python_wrapper.h"

const float table_dist_from_robot = 0.3;
const float table_width = 1, table_length = 1;
const float table_height = 0.8;
const float rope_len = 1.5;
const int rope_segments = 150;
static vector<btVector3> initTableCornersWorld() {
  vector<btVector3> v;
  v.push_back(METERS * btVector3(table_dist_from_robot, -table_width/2, table_height));
  v.push_back(METERS * btVector3(table_dist_from_robot, table_width/2, table_height));
  v.push_back(METERS * btVector3(table_dist_from_robot + table_length, table_width/2, table_height));
  v.push_back(METERS * btVector3(table_dist_from_robot + table_length, -table_width/2, table_height));
  return v;
}

static vector<btVector3> initRopeControlPointsWorld(const btVector3 &startPt, const btVector3 &endPt) {
  vector<btVector3> v;
  for (int i = 0; i < rope_segments; ++i) {
    float a = (float)i / rope_segments;
    v.push_back((1.-a)*startPt + a*endPt);
  }
  return v;
}

static btVector3 randPtWithinBounds(const btVector3 &minv, const btVector3 &maxv) {
  return btVector3(
    (float)rand()/RAND_MAX*(maxv.x() - minv.x()) + minv.x(),
    (float)rand()/RAND_MAX*(maxv.y() - minv.y()) + minv.y(),
    (float)rand()/RAND_MAX*(maxv.z() - minv.z()) + minv.z()
  );
}

static void moveRope(const btVector3 &tableMin, const btVector3 &tableMax, TableRopeScene &scene) {
/*  btVector3 ropeStart = randPtWithinBounds(tableMin, tableMax);
  btVector3 ropeMiddle = randPtWithinBounds(tableMin, tableMax);
  btVector3 ropeEnd = randPtWithinBounds(tableMin, tableMax);*/
  /*btVector3 ropeStart = tableMin + METERS*btVector3(0.01, 0.01, 0);
  btVector3 ropeMiddle = btVector3((tableMin.x() + tableMax.x())/2., tableMax.y()-0.01*METERS, tableMin.z());
  btVector3 ropeEnd = btVector3(tableMax.x() - 0.01*METERS, tableMin.y() + 0.01*METERS, tableMin.z());*/
  btVector3 ropeStart = btVector3(tableMax.x() - 0.1*METERS, tableMax.y() - 0.1*METERS, tableMin.z());
  btVector3 ropeMiddle = btVector3((tableMin.x() + tableMax.x())/2., tableMin.y()+0.1*METERS, tableMin.z());
  btVector3 ropeEnd = btVector3(tableMin.x() + 0.1*METERS, tableMax.y()-0.1*METERS, tableMin.z());
  // constrain the first and last points of the rope to be in the randomly chosen
  // locations on the table
  CapsuleRope::Ptr rope = scene.getRope();
  boost::shared_ptr<btRigidBody> rbStart = rope->getChildren()[0]->rigidBody;
  boost::shared_ptr<btRigidBody> rbMid = rope->getChildren()[rope->getChildren().size()/2]->rigidBody;
  boost::shared_ptr<btRigidBody> rbEnd = rope->getChildren()[rope->getChildren().size() - 1]->rigidBody;
  btTransform tStart(rbStart->getCenterOfMassTransform()); tStart.setOrigin(ropeStart);
  btTransform tMid(rbMid->getCenterOfMassTransform()); tMid.setOrigin(ropeMiddle);
  btTransform tEnd(rbEnd->getCenterOfMassTransform()); tEnd.setOrigin(ropeEnd);
  // temporarily turn off gravity
  scene.bullet->setGravity(btVector3(0, 0, 0));
  for (int i = 0; i < 1000; ++i) {
    // if any point in the rope is off the table, nudge it inwards
    /*
    for (int z = 0; z < rope->getChildren().size(); ++z) {
      btVector3 origin = rope->getChildren()[z].getCenterOfMassTransform().getOrigin();
      if (origin.x() > tableMax
    }*/
    rbStart->setCenterOfMassTransform(tStart);
    rbMid->setCenterOfMassTransform(tMid);
    rbEnd->setCenterOfMassTransform(tEnd);
    scene.step(DT);
  }
  scene.bullet->setGravity(BulletConfig::gravity);
  scene.stepFor(DT, 0.25);
}

static void moveRopeVert(const btVector3 &tableMin, const btVector3 &tableMax, TableRopeScene &scene) {
  btVector3 ropeStart(tableMin.x() + 0.1*METERS, tableMin.y() + 0.4*METERS, tableMin.z());
  btVector3 ropeMiddle(tableMax.x() - 0.1*METERS, 0.5*(tableMin.y() + tableMax.y()), tableMin.z());
  btVector3 ropeEnd(tableMin.x() + 0.1*METERS, tableMax.y() - 0.4*METERS, tableMin.z());
  // constrain the first and last points of the rope to be in the randomly chosen
  // locations on the table
  CapsuleRope::Ptr rope = scene.getRope();
  boost::shared_ptr<btRigidBody> rbStart = rope->getChildren()[0]->rigidBody;
  boost::shared_ptr<btRigidBody> rbMid = rope->getChildren()[rope->getChildren().size()/2]->rigidBody;
  boost::shared_ptr<btRigidBody> rbEnd = rope->getChildren()[rope->getChildren().size() - 1]->rigidBody;
  btTransform tStart(rbStart->getCenterOfMassTransform()); tStart.setOrigin(ropeStart);
  btTransform tMid(rbMid->getCenterOfMassTransform()); tMid.setOrigin(ropeMiddle);
  btTransform tEnd(rbEnd->getCenterOfMassTransform()); tEnd.setOrigin(ropeEnd);
  // temporarily turn off gravity
  scene.bullet->setGravity(btVector3(0, 0, 0));
  for (int i = 0; i < 1000; ++i) {
    rbStart->setCenterOfMassTransform(tStart);
    rbMid->setCenterOfMassTransform(tMid);
  //  rbEnd->setCenterOfMassTransform(tEnd);
    scene.step(DT);
  }
  for (int i = 0; i < 1000; ++i) {
    rbStart->setCenterOfMassTransform(tStart);
    rbMid->setCenterOfMassTransform(tMid);
    rbEnd->setCenterOfMassTransform(tEnd);
    scene.step(DT);
  }
  scene.bullet->setGravity(BulletConfig::gravity);
  scene.stepFor(DT, 0.25);
}

struct LocalConfig : public Config {
  static float pert;
  static string rope;
  LocalConfig() : Config() { 
    params.push_back(new Parameter<float>("pert", &pert, "rope perturbation variance"));
    params.push_back(new Parameter<string>("rope", &rope, "rope control points file"));
  }
};
float LocalConfig::pert = 1.;
string LocalConfig::rope;

int main(int argc, char *argv[]) {
  GeneralConfig::scale = 10.;
  BulletConfig::maxSubSteps = 0;
  BulletConfig::gravity = btVector3(0, 0, -1);

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SceneConfig());
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);

  LoggingInit();
  Python_setup();

  // Initialize scene
  const btVector3 ropeStartPt = METERS * btVector3(table_dist_from_robot + table_length/2, 0, table_height + 0.1);
  const btVector3 ropeEndPt = METERS * btVector3(table_dist_from_robot + table_length/2, rope_len, table_height + 0.1);
  const btVector3 finalRopeStartPt = METERS * btVector3(table_dist_from_robot + table_length/2, 0, table_height + 0.1);
  const btVector3 finalRopeEndPt = METERS * btVector3(table_dist_from_robot + table_length/2, rope_len, table_height + 0.1);
  vector<btVector3> tableCorners = initTableCornersWorld();

  vector<btVector3> ropeCtlPts = toBulletVectors(floatMatFromFile(LocalConfig::rope)) * METERS;

  if (LocalConfig::pert <= 0.00001) {
    try {
      lfd::CurvePerturbation cpert;
      ropeCtlPts = cpert.perturbCurve(ropeCtlPts, LocalConfig::pert);
    } catch (const py::error_already_set &e) {
      PyErr_Print();
      throw e;
    }
  }

  TableRopeScene scene(tableCorners, ropeCtlPts);
  scene.m_table->rigidBody->setFriction(0.9);
  cout << "defualt margin " << 
    scene.m_table->rigidBody->getCollisionShape()->getMargin() << endl;
  scene.m_table->rigidBody->getCollisionShape()->setMargin(0.1);
  scene.startViewer();
  scene.stepFor(DT, 1);

/*  btVector3 minRopePos = tableCorners[0];
  btVector3 maxRopePos = tableCorners[2]; maxRopePos.setX(0.5*(maxRopePos.x() + minRopePos.x()));
  moveRopeVert(minRopePos, maxRopePos, scene);*/

  scene.viewer.frame();
  scene.setDrawing(true);

  lfd::TaskExecuter ex(scene);
  try {
    ex.run();
  } catch (const py::error_already_set &e) {
    PyErr_Print();
    throw e;
  }

  scene.startFixedTimestepLoop(DT);
  return 0;
}
