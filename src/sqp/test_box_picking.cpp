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

osgText::Font* font = osgText::readFontFile("fonts/arial.ttf");
class PlotText : public PlotObject {
public:
  typedef boost::shared_ptr<PlotText> Ptr;
  PlotText() {
    m_geode = new osg::Geode();

    osgText::Text* text = new osgText::Text;
    float margin = 50;
    float windowHeight = 500;
    text->setFont(font);
    text->setColor(osg::Vec4(1,1,1,1));
    text->setCharacterSize(100);
    text->setPosition(osg::Vec3(margin, windowHeight - margin, 0.0f));
    text->setLayout(osgText::Text::LEFT_TO_RIGHT);
    text->setText("text->setLayout(osgText::Text::LEFT_TO_RIGHT);");
    m_geode->addDrawable(text);
  }
};

double ang = 0;
RaveObject::Ptr roller;

void step() {
  roller->body->SetDOFValues(vector<double> (roller->body->GetDOF(), ang));
  getGlobalScene()->env->step(.025, 1, .005);
  ang -= .03;
}

void step(int n) {
  getGlobalScene()->step(0);
  for (int i = 0; i < n; ++i)
    step();
}

void execTraj(const MatrixXd& traj, RaveRobotObject::Manipulator::Ptr arm, Scene& scene) {
  for (int i = 0; i < traj.rows(); ++i) {
    arm->setDOFValues(toDoubleVec(traj.row(i)));
    step(20);
    if (SQPConfig::pauseEachIter) scene.idle(true);
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

  LoadFromRave(scene.env, scene.rave, false);
  RobotManager rm(scene);

  vector<KinBodyPtr> bodies;
  penv->GetBodies(bodies);
  BOOST_FOREACH(KinBodyPtr body, bodies) {
    cout << body->GetName() << endl;
    BOOST_FOREACH(KinBody::LinkPtr link, body->GetLinks()) {
      link->SetStatic(body->GetName().substr(0, 7) != "pickbox");
      cout << link->GetName() << " " << link->IsStatic() << " " << link->GetMass() << endl;
    }
  }
  Scene scene2(penv);
  double origLinkPadding = BulletConfig::linkPadding;
  BulletConfig::linkPadding = 0;
  LoadFromRave(scene2.env, scene2.rave, true);
  BulletConfig::linkPadding = origLinkPadding;
  RobotManager rm2(scene2);
  rm2.bot->setColor(0, 0, 1, .8);


  cv::Mat ipiLogo = cv::imread("/home/joschu/Dropbox/Proj/ipi/ipilogo400x343.png");
  assert(!ipiLogo.empty());

  roller = getObjectByName(scene2.env, scene2.rave, "Roller");
  BOOST_FOREACH(BulletObjectPtr child, roller->children) {
    child->setTexture(ipiLogo);
    child->rigidBody->setAngularVelocity(btVector3(0,1,0));
  }

  ArmPrinter ap(rm2.botLeft, rm2.botRight);
  scene2.addVoidKeyCallback('c', boost::bind(&ArmPrinter::printCarts, &ap), "print cart");
  scene2.addVoidKeyCallback('j', boost::bind(&ArmPrinter::printJoints, &ap), "print joints");
  scene2.addVoidKeyCallback('a', boost::bind(&ArmPrinter::printAll, &ap), "print all dofs");

  vector<RaveObject::Ptr> boxes, boxes2;

  osg::Depth* depth = new osg::Depth;
  depth->setWriteMask(false);
  for (int i = 0; i < scene.env->objects.size(); ++i) {
    RaveObject::Ptr maybeRO = boost::dynamic_pointer_cast<RaveObject>(scene.env->objects[i]);
    RaveObject::Ptr maybeRO2 = boost::dynamic_pointer_cast<RaveObject>(scene2.env->objects[i]);
    if (maybeRO && maybeRO->body->GetName().substr(0, 7) == "pickbox") {
      maybeRO2->setTexture(ipiLogo);
      boxes.push_back(maybeRO);
      boxes2.push_back(maybeRO2);
      maybeRO2->children[0]->rigidBody->setDamping(.9,.95);
    }
    else {
      if (maybeRO && maybeRO->body->GetName().substr(0,3)=="box") maybeRO2->setColor(1,1,1,.4);
    }
  }

  btTransform tDrop = btTransform(btQuaternion(0.5, -0.5, 0.5, 0.5), btVector3(.5, -.2, 1.2));

  drawAxes(scaleTransform(tDrop, METERS), .1 * METERS, scene.env);
  float bside = .35;
  float bheight = .30;

  setGlobalScene(&scene2);
  setGlobalEnv(scene2.env);
  if (GeneralConfig::verbose > 0)
    getGRBEnv()->set(GRB_IntParam_OutputFlag, 0);
//  scene.startViewer();
  scene2.startViewer();
  scene2.idle(true);

//  PlotText::Ptr plotText(new PlotText());
//  scene2.env->osg->root->addChild(plotText->getOSGNode());

  removeBodiesFromBullet(rm.bot->children, scene.env->bullet->dynamicsWorld);

  //  boost::thread physThread(&physicsLoop, scene2);
  step(30);
  for (int iBox = boxes.size() - 1; iBox >= 0; --iBox) {
    RaveObject::Ptr curBox = boxes[iBox];
    {
      btTransform goalTrans = frontPickPose(toBtVector(curBox->body->GetTransform().trans), btVector3(bside, bside,
                                                                                                      bheight) / 2);
      PlanningProblem prob;
      prob.addPlotter(ArmPlotterPtr(new ArmPlotter(rm.botRight, &scene, SQPConfig::plotDecimation)));
      planArmToCartTarget(prob, toVectorXd(rm.botRight->getDOFValues()), goalTrans, rm.botRight, false);
      prob.optimize(50);
      rm.botRight->setDOFValues(toDoubleVec(prob.m_currentTraj.row(prob.m_currentTraj.rows() - 1)));
      execTraj(prob.m_currentTraj, rm2.botRight, scene2);
      updateRaveFromBullet(scene2.rave, scene2.env);
      updateBulletFromRave(scene.rave, scene.env);
    }
    rm.botRight->manip->GetRobot()->Grab(curBox->body, rm.botRight->manip->GetEndEffector());
    boxes2[iBox]->is_dynamic = false;
    boxes2[iBox]->children[0]->setKinematic(true);
    setGrabberLink(rm.botRight->manip->GetEndEffector(), curBox->body);
    {
      PlanningProblem prob;
      prob.addPlotter(ArmPlotterPtr(new ArmPlotter(rm.botRight, &scene, SQPConfig::plotDecimation)));
      planArmToCartTarget(prob, toVectorXd(rm.botRight->getDOFValues()), tDrop, rm.botRight, false);
      prob.optimize(50);
      execTraj(prob.m_currentTraj, rm2.botRight, scene2);
      updateRaveFromBullet(scene2.rave, scene2.env);
      updateBulletFromRave(scene.rave, scene.env);
    }

    rm.botRight->manip->GetRobot()->ReleaseAllGrabbed();
    boxes2[iBox]->is_dynamic = true;
    boxes2[iBox]->children[0]->setKinematic(false);
    //    idleTwoScenes(scene, scene2);

    static int boxcount=0;
    if (boxes.size() - iBox >  6){
      scene.env->remove(boxes[iBox+6]);
      scene2.env->remove(boxes2[iBox+6]);
    }
    //    sleep(.4);
    step(100);

  }

}
