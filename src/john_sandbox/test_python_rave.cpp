#include <boost/python.hpp>
#include <openrave/openrave.h>
#include "simulation/simplescene.h"
#include "utils/config.h"
#include "robots/robot_manager.h"
#include "sqp/planning_problems.h"
#include "sqp/sqp_algorithm.h"
#include "sqp/config_sqp.h"
#include "simulation/util.h"
#include "simulation/bullet_io.h"
#include "sqp/plotters.h"
#include "sqp/kinematics_utils.h"
#include <osg/Depth>
using namespace OpenRAVE;
using namespace std;
using namespace util;
using namespace Eigen;
namespace py = boost::python;

btTransform frontPickPose(const btVector3& center, const btVector3& halfExtents) {
  float eebox=.2;
  btMatrix3x3 rot(0, 0, 1,
                  0, -1, 0,
                  1, 0, 0);
  btVector3 pos(center.x() - halfExtents.x() - (eebox + .03), center.y(), center.z());
  return btTransform(rot, pos);
}

void execTraj(const MatrixXd& traj, RaveRobotObject::Manipulator::Ptr arm) {
  for (int i=0; i < traj.rows(); ++i) {
    arm->setDOFValues(toDoubleVec(traj.row(i)));
    getGlobalScene()->step(.05,10,.01);
    sleep(.03);
  }
}

int main(int argc, char* argv []) {

  GeneralConfig::scale=10;
  SQPConfig::distContSafe = -.1;
  SQPConfig::distDiscSafe = .01;
  SQPConfig::distPen = .02;
  BulletConfig::linkPadding = .02;
  BulletConfig::margin = .001;
  SQPConfig::padMult=2;

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(SQPConfig());
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);

  Py_Initialize();
  py::object main_module = py::import("__main__");
  py::object main_namespace = main_module.attr("__dict__");

  try {
    py::exec(
        "import sys,os;"
        "sys.path.append('/home/joschu/Dropbox/Proj/ipi/openrave');"
        "os.chdir('/home/joschu/Dropbox/Proj/ipi/openrave');"
        "from loadtest import *;",
        main_namespace);
  }
  catch (py::error_already_set err) {
    PyErr_Print();
    PyErr_Clear();
    throw;
  }
  EnvironmentBasePtr penv = OpenRAVE::RaveGetEnvironment(1);
  assert(penv);

  vector<KinBodyPtr> bodies;
  penv->GetBodies(bodies);
  BOOST_FOREACH(KinBodyPtr body, bodies) {
    if (body->GetName().substr(0,3) == "box") {
      BOOST_FOREACH(KinBody::LinkPtr link, body->GetLinks()) {
//        link->SetStatic(false);
      }
    }
  }

  Scene scene(penv);
  scene.addVoidKeyCallback('=', boost::bind(&adjustWorldTransparency, .05), "increase opacity");
  scene.addVoidKeyCallback('-', boost::bind(&adjustWorldTransparency, -.05), "decrease opacity");

  LoadFromRave(scene.env, scene.rave);
  RobotManager rm(scene);
  rm.bot->setColor(0,0,1,.8);

  vector<RaveObject::Ptr> boxes;

  osg::Depth* depth = new osg::Depth;
  depth->setWriteMask( false );
  cv::Mat ipiLogo = cv::imread("/home/joschu/Dropbox/Proj/ipi/ipilogo400x343.png");
  assert(!ipiLogo.empty());
  int iBox=0;
  BOOST_FOREACH(EnvironmentObject::Ptr obj, scene.env->objects) {
    RaveObject::Ptr maybeRO = boost::dynamic_pointer_cast<RaveObject>(obj);
    if (maybeRO && maybeRO->body->GetName().substr(0,3) == "box") {
      if (iBox++>3) {
        maybeRO->setTexture(ipiLogo);
        boxes.push_back(maybeRO);
        cout << "mass:" << 1/maybeRO->associatedObj(maybeRO->body->GetLinks()[0])->rigidBody->getInvMass() << endl;
      }
      else {
        maybeRO->setColor(1,1,1,.3);
        BOOST_FOREACH(BulletObject::Ptr child, maybeRO->children)
          child->node->getOrCreateStateSet()->setAttributeAndModes( depth, osg::StateAttribute::ON );
      }
    }
  }


  btTransform tDrop(
      btMatrix3x3(-1,0,0,
                  0,1,0,
                  0,0,-1),
      btVector3(.5, -.6, 1.3));
drawAxes(scaleTransform(tDrop, METERS), .1*METERS, scene.env);
  float bside = .35;
  float bheight=.30;

  setGlobalScene(&scene);
  setGlobalEnv(scene.env);
  if (GeneralConfig::verbose > 0) getGRBEnv()->set(GRB_IntParam_OutputFlag, 0);
  scene.startViewer();

  removeBodiesFromBullet(rm.bot->children, scene.env->bullet->dynamicsWorld);


  BOOST_REVERSE_FOREACH(RaveObject::Ptr curBox, boxes) {
    {
      btTransform goalTrans = frontPickPose(toBtVector(curBox->body->GetTransform().trans), btVector3(bside, bside,
                                                                                                      bheight) / 2);
      PlanningProblem prob;
      prob.addPlotter(ArmPlotterPtr(new ArmPlotter(rm.botRight, &scene, SQPConfig::plotDecimation)));
      planArmToCartTarget(prob, toVectorXd(rm.botRight->getDOFValues()), goalTrans, rm.botRight);
      rm.botRight->setDOFValues(toDoubleVec(prob.m_currentTraj.row(prob.m_currentTraj.rows() - 1)));
      execTraj(prob.m_currentTraj, rm.botRight);
    }
    rm.botRight->manip->GetRobot()->Grab(curBox->body, rm.botRight->manip->GetEndEffector());
    setGrabberLink(rm.botRight->manip->GetEndEffector(), curBox->body);
    {
      PlanningProblem prob;
      prob.addPlotter(ArmPlotterPtr(new ArmPlotter(rm.botRight, &scene, SQPConfig::plotDecimation)));
      planArmToCartTarget(prob, toVectorXd(rm.botRight->getDOFValues()), tDrop, rm.botRight);
      execTraj(prob.m_currentTraj, rm.botRight);
    }
    rm.botRight->manip->GetRobot()->ReleaseAllGrabbed();
    scene.env->remove(curBox);

  }

  scene.startLoop();


}
