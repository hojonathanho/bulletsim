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
using namespace OpenRAVE;
using namespace std;
using namespace util;
using namespace Eigen;
namespace py = boost::python;

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

btTransform topDrop() {
/*array([[-1. ,  0. ,  0. ,  0.5],
       [ 0. , -1. ,  0. , -0.6],
       [ 0. ,  0. , -1. ,  1.3],
       [ 0. ,  0. ,  0. ,  1. ]])
 *
 */
  btMatrix3x3 rot(-1, 0, 0, 0, -1, 0, 0, 0, -1);
  btVector3 pos(.5, -.6, 1.3);
  return btTransform(rot, pos);
}

btTransform sideDrop() {
  /*
   * array([[ 0. , -1. ,  0. ,  0.5],
       [ 0. ,  0. , -1. , -0.2],
       [ 1. ,  0. ,  0. ,  1.2],
       [ 0. ,  0. ,  0. ,  1. ]])
   */
  btMatrix3x3 rot(0, -1, 0, 0, 0, -1, 1, 0, 0);
  btQuaternion q; rot.getRotation(q);
  btVector3 pos(.5, -.2, 1.2);
  return btTransform(rot, pos);

}

btTransform boxTransform() {
//  array([[  2.220e-15,   1.000e+00,   8.332e-16,   5.000e-01],
//         [ -1.000e+00,   2.220e-15,  -1.679e-15,  -5.850e-01],
//         [ -1.679e-15,  -8.332e-16,   1.000e+00,   1.200e+00],
//         [  0.000e+00,   0.000e+00,   0.000e+00,   1.000e+00]])
  btMatrix3x3 rot(0,1,0,  -1, 0, 0,  0, 0, 1);
  btVector3 pos(.8, -.585, 1.2);
  return btTransform(rot, pos);

}

void execTraj(const MatrixXd& traj, RaveRobotObject::Manipulator::Ptr arm, Scene& scene) {
  for (int i = 0; i < traj.rows(); ++i) {
    arm->setDOFValues(toDoubleVec(traj.row(i)));
    scene.step(0);
    if (SQPConfig::pauseEachIter) pauseScene();
  }
}

void updateRaveFromBullet(RaveInstancePtr rave, EnvironmentPtr env) {
  typedef std::map<KinBodyPtr, RaveObject*>::value_type RaveBullet;
  BOOST_FOREACH(RaveBullet rb, rave->rave2bulletsim) {
    if (!rb.first->IsRobot())
      rb.first->SetTransform(toRaveTransform(rb.second->children[0]->rigidBody->getCenterOfMassTransform(), 1 / METERS));
  }
}
void updateBulletFromRave(RaveInstancePtr rave, EnvironmentPtr env) {
  typedef std::map<KinBodyPtr, RaveObject*>::value_type RaveBullet;
  BOOST_FOREACH(RaveBullet rb, rave->rave2bulletsim) {
    rb.second->updateBullet();
  }
}

int main(int argc, char* argv[]) {

  GeneralConfig::verbose=20000;
  GeneralConfig::scale = 1;
  SQPConfig::distContSafe = -.1;
  SQPConfig::distDiscSafe = .02;
  SQPConfig::distPen = .04;
  SQPConfig::collCoefInit=100;
  SQPConfig::padMult = 2;
  BulletConfig::margin = .01;
  BulletConfig::gravity=btVector3(0,0,-1);
  BulletConfig::linkPadding = .05;
  BulletConfig::friction=100;

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

  Scene scene(penv);
  scene.addVoidKeyCallback('=', boost::bind(&adjustWorldTransparency, .05), "increase opacity");
  scene.addVoidKeyCallback('-', boost::bind(&adjustWorldTransparency, -.05), "decrease opacity");

  LoadFromRave(scene.env, scene.rave);
  RobotManager rm(scene);

  vector<KinBodyPtr> bodies;
  penv->GetBodies(bodies);
  BOOST_FOREACH(KinBodyPtr body, bodies) {
    BOOST_FOREACH(KinBody::LinkPtr link, body->GetLinks()) {
      link->SetStatic(body->GetName().substr(0, 7) != "pickbox");
    }
  }

  cv::Mat ipiLogo = cv::imread("/home/joschu/Dropbox/Proj/ipi/smiley.jpg");
  assert(!ipiLogo.empty());

  ArmPrinter ap(rm.botLeft, rm.botRight);
  scene.addVoidKeyCallback('c', boost::bind(&ArmPrinter::printCarts, &ap), "print cart");
  scene.addVoidKeyCallback('j', boost::bind(&ArmPrinter::printJoints, &ap), "print joints");
  scene.addVoidKeyCallback('a', boost::bind(&ArmPrinter::printAll, &ap), "print all dofs");

  vector<RaveObject::Ptr> boxes;

  for (int i = 0; i < scene.env->objects.size(); ++i) {
    RaveObject::Ptr maybeRO = boost::dynamic_pointer_cast<RaveObject>(scene.env->objects[i]);
    if (maybeRO && maybeRO->body->GetName().substr(0, 7) == "pickbox") {
      maybeRO->setTexture(ipiLogo);
      boxes.push_back(maybeRO);
    }
    else {
      if (maybeRO && maybeRO->body->GetName().substr(0,3)=="box") maybeRO->setColor(1,1,1,.4);
    }
  }



  LOG_INFO("top drop: " << topDrop());
  LOG_INFO("side drop: " << sideDrop());

  const float bside = .35;
  const float bheight = .30;

  setGlobalScene(&scene);
  setGlobalEnv(scene.env);
  if (GeneralConfig::verbose > 0)
    getGRBEnv()->set(GRB_IntParam_OutputFlag, 0);
//  scene.startViewer();
  scene.startViewer();
  scene.idle(true);

  removeBodiesFromBullet(rm.bot->children, scene.env->bullet->dynamicsWorld);

  ArmPlotterPtr plotter(new ArmPlotter(rm.botRight, &scene, SQPConfig::plotDecimation));
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
      execTraj(opt.m_traj, rm.botRight, scene);
    }

    rm.bot->grab(curBox, rm.botRight->manip->GetEndEffector());

    {

      TrajOptimizer opt;
      opt.m_plotters.push_back(plotter);
//      util::drawAxes(util::toBtTransform(curBox->body->GetLinks()[0]->GetTransform(),METERS), .1, getGlobalEnv());
//      util::drawAxes(util::scaleTransform(boxTransform(), METERS), .3, getGlobalEnv());
      LOG_INFO("trying ik for side drop");
      bool success = setupArmToCartTarget(opt, boxTransform(), rm.botRight, curBox->body->GetLinks()[0]);
      assert(success);

      opt.addCost(CostPtr(new ThisSideUpCost(&opt, rm.bot->robot, rm.botRight->manip->GetEndEffector(), rm.botRight->manip->GetArmIndices(), false, 1, false)));
//      opt.addCost(CostPtr(new CartAccCost(&opt, rm.bot->robot, rm.botRight->manip->GetEndEffector(), rm.botRight->manip->GetArmIndices(), false, 100)));

      TrajOptimizer::OptStatus status = opt.optimize();
      opt.m_plotters[0]->clear();
      execTraj(opt.m_traj, rm.botRight, scene);
    }

    rm.botRight->manip->GetRobot()->ReleaseAllGrabbed();

    scene.env->remove(curBox);


  }

}
