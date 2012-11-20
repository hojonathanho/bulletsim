#include <boost/python.hpp>
#include <openrave/openrave.h>
#include "simulation/simplescene.h"
#include "utils/config.h"
#include "robots/robot_manager.h"
#include "sqp/config_sqp.h"
#include "simulation/util.h"
#include "simulation/bullet_io.h"
#include "sqp/plotters.h"
#include "sqp/kinematics_utils.h"
#include "sqp/planning_problems2.h"
#include "sqp/traj_sqp.h"
#include <osg/Depth>
#include <opencv2/highgui/highgui.hpp>
#include "utils/clock.h"
#include <osgDB/ReadFile>

using namespace OpenRAVE;
using namespace std;
using namespace util;
using namespace Eigen;
namespace py = boost::python;

const float bside = .35;
const float bheight = .30;

btTransform frontPickPose(const btVector3& center, const btVector3& halfExtents) {
  float eebox = .2;
  btMatrix3x3 rot(0, 0, 1, 0, -1, 0, 1, 0, 0);
  btVector3 pos(center.x() - halfExtents.x() - (eebox + .03), center.y(), center.z());
  return btTransform(rot, pos);
}

btTransform topPickPose(const btVector3& center, const btVector3& halfExtents) {
  float eebox = .2;
  btMatrix3x3 rot(1, 0, 0, 0, -1, 0, 0, 0, -1);
  btVector3 pos(center.x(), center.y(), center.z() + halfExtents.z() + (eebox + .03));
  return btTransform(rot, pos);
}

#if 0
btTransform boxTransform() {
//  array([[  2.220e-15,   1.000e+00,   8.332e-16,   5.000e-01],
//         [ -1.000e+00,   2.220e-15,  -1.679e-15,  -5.850e-01],
//         [ -1.679e-15,  -8.332e-16,   1.000e+00,   1.200e+00],
//         [  0.000e+00,   0.000e+00,   0.000e+00,   1.000e+00]])
  btMatrix3x3 rot(0,1,0,  -1, 0, 0,  0, 0, 1);
  btVector3 pos(.8, -.585, 1.2);
  return btTransform(rot, pos);

}
#endif

btTransform robotTransform;
btTransform frontDropPose() {
//  [[ -2.31316077e-16   4.42679879e-17  -1.00000000e+00   4.85722573e-17]
//   [ -5.55111512e-17   1.00000000e+00   2.08166817e-17  -6.50000000e-01]
//   [  1.00000000e+00   5.55111512e-17  -2.08166817e-16   2.22044605e-16]
//   [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
  btMatrix3x3 rot(0,0,-1,  0, 1, 0,  1, 0, 0);
  btVector3 pos(0, -.65, 0);
  return robotTransform * btTransform(rot, pos);

}

btTransform topDropPose() {
  return btTransform(btQuaternion(0.996599, -0.0426953, -0.0692106, 0.0133295), btVector3(0.305763, -0.830994, 1.22276));

}


void execTraj(const MatrixXd& traj, RaveRobotObject::Manipulator::Ptr arm, Scene& scene) {
  for (int i = 0; i < traj.rows(); ++i) {
    arm->setDOFValues(toDoubleVec(traj.row(i)));
    scene.step(0);
    if (SQPConfig::pauseEachIter) pauseScene();
  }
}


int main(int argc, char* argv[]) {

  GeneralConfig::verbose=20000;
  GeneralConfig::scale = 100;
  SQPConfig::distContSafe = -.1;
  SQPConfig::distDiscSafe = .025;
  BulletConfig::linkPadding = .06;
  SQPConfig::distPen = .06;
  SQPConfig::collCoefInit=100;
  SQPConfig::padMult = 2;
  BulletConfig::margin = 0;
//  BulletConfig::gravity=btVector3(0,0,-1);
//  BulletConfig::friction=100;

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(SQPConfig());
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);

  initializeGRB();


  Py_Initialize();
  py::object main_module = py::import("__main__");
  py::object main_namespace = main_module.attr("__dict__");

  try {
    py::exec("import sys,os;"
      "sys.path.append('/home/joschu/Dropbox/Proj/ipi/openrave');"
      "os.chdir('/home/joschu/Dropbox/Proj/ipi/openrave');"
      "from loadtest import *;", main_namespace);
  } catch (py::error_already_set err) {
    PyErr_Print();
    PyErr_Clear();
    throw;
  }
  EnvironmentBasePtr penv = OpenRAVE::RaveGetEnvironment(1);
  assert(penv);

//#define PPSCENE
#ifdef PPSCENE
  PhysicsPlanningScene ppscene(penv);
  Scene& scene = *ppscene.m_planScene;
  ppscene.startViewer();
#else
  Scene scene(penv);
  LoadFromRave(scene.env, scene.rave);
  setGlobalScene(&scene);
#endif
  getGlobalScene()->addVoidKeyCallback('=', boost::bind(&adjustWorldTransparency, .05), "increase opacity");
  getGlobalScene()->addVoidKeyCallback('-', boost::bind(&adjustWorldTransparency, -.05), "decrease opacity");

  RobotManager rm(scene);

  vector<KinBodyPtr> bodies;
  penv->GetBodies(bodies);
  BOOST_FOREACH(KinBodyPtr body, bodies) {
    BOOST_FOREACH(KinBody::LinkPtr link, body->GetLinks()) {
      link->SetStatic(body->GetName().substr(0, 7) != "pickbox");
    }
  }


  osg::ref_ptr<osg::Image> image = osgDB::readImageFile("/home/joschu/Dropbox/Proj/ipi/smiley1.jpg");
  ArmPrinter ap(rm.botLeft, rm.botRight);
  getGlobalScene()->addVoidKeyCallback('c', boost::bind(&ArmPrinter::printCarts, &ap), "print cart");
  getGlobalScene()->addVoidKeyCallback('j', boost::bind(&ArmPrinter::printJoints, &ap), "print joints");
  getGlobalScene()->addVoidKeyCallback('a', boost::bind(&ArmPrinter::printAll, &ap), "print all dofs");


  vector<RaveObject::Ptr> boxes;
#ifdef PPSCENE
  vector<RaveObject::Ptr> ppboxes;
#endif

  for (int i = 0; i < scene.env->objects.size(); ++i) {
    RaveObject::Ptr maybeRO = boost::dynamic_pointer_cast<RaveObject>(scene.env->objects[i]);
    if (maybeRO && maybeRO->body->GetName().substr(0, 7) == "pickbox") {
      maybeRO->children[0]->setTexture(image);
      boxes.push_back(maybeRO);
    }
    else {
      if (maybeRO && maybeRO->body->GetName().substr(0,3)=="box") maybeRO->setColor(1,1,1,.4);
    }
#ifdef PPSCENE
    {RaveObject::Ptr maybeRO = boost::dynamic_pointer_cast<RaveObject>(ppscene.env->objects[i]);
    if (maybeRO && maybeRO->body->GetName().substr(0, 7) == "pickbox") {
      maybeRO->children[0]->setTexture(image);
      ppboxes.push_back(maybeRO);
    }
    else {
      if (maybeRO && maybeRO->body->GetName().substr(0,3)=="box") maybeRO->setColor(1,1,1,.4);
    }}
#endif
  }

 robotTransform = util::toBtTransform(rm.bot->robot->GetTransform());
  getGlobalScene()->idle(true);



#ifndef PPSCENE
  setGlobalScene(&scene);
  setGlobalEnv(scene.env);
  scene.startViewer();
#endif


#if 0
  while (true) {
    scene.step(0);
    sleep(.05);
  }
#endif

  removeBodiesFromBullet(rm.bot->children, scene.env->bullet->dynamicsWorld);

#ifdef PPSCENE
  RobotManager rm2(ppscene);
  ArmPlotterPtr plotter(new ArmPlotter(rm2.botRight, util::getGlobalScene(), SQPConfig::plotDecimation));
#else
  ArmPlotterPtr plotter(new ArmPlotter(rm.botRight, util::getGlobalScene(), SQPConfig::plotDecimation));
#endif
  for (int iBox = boxes.size() - 1; iBox >= 0; --iBox) {
    RaveObject::Ptr curBox = boxes[iBox];
    btVector3 boxpos = toBtVector(curBox->body->GetTransform().trans);
    bool useFront = (boxpos.z() > 1.7);


    {
      btTransform goalTrans = useFront ?
          frontPickPose(boxpos, btVector3(bside, bside, bheight) / 2)
        : topPickPose(boxpos, btVector3(bside, bside, bheight) / 2);

      TrajOptimizer opt;
      opt.m_plotters.push_back(plotter);
      bool success = setupArmToCartTarget(opt, goalTrans, rm.botRight);
      assert(success);
      TrajOptimizer::OptStatus status = opt.optimize();
      opt.m_plotters[0]->clear();
      execTraj(opt.m_traj, rm.botRight, *getGlobalScene());
    }

    rm.bot->grab(curBox, rm.botRight->manip->GetEndEffector());

    {

      TrajOptimizer opt;
      opt.m_plotters.push_back(plotter);
//      util::drawAxes(util::toBtTransform(curBox->body->GetLinks()[0]->GetTransform(),METERS), .1, getGlobalEnv());
//      util::drawAxes(util::scaleTransform(boxTransform(), METERS), .3, getGlobalEnv());
      LOG_INFO("trying ik for side drop");

      btTransform dropPose = useFront ? frontDropPose() : topDropPose();

      bool success = setupArmToCartTarget(opt, dropPose, rm.botRight, rm.botRight->manip->GetEndEffector());
      assert(success);

//      opt.addCost(CostPtr(new ThisSideUpCost(&opt, rm.bot->robot, rm.botRight->manip->GetEndEffector(), rm.botRight->manip->GetArmIndices(), false, 1, false)));
      opt.addCost(CostPtr(new CartAccCost(&opt, rm.bot->robot, rm.botRight->manip->GetEndEffector(), rm.botRight->manip->GetArmIndices(), false, 100)));

      TrajOptimizer::OptStatus status = opt.optimize();
      opt.m_plotters[0]->clear();
      execTraj(opt.m_traj, rm.botRight, *getGlobalScene());
    }

    rm.botRight->manip->GetRobot()->ReleaseAllGrabbed();

    scene.env->remove(curBox);
#ifdef PPSCENE
    ppscene.env->remove(ppboxes[iBox]);
#endif

  }

}
