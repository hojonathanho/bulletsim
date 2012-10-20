#include "simulation/simplescene.h"
#include "plotters.h"
#include "traj_costs.h"
#include "utils/logging.h"
#include "kinematics_utils.h"
#include "state_setter.h"
#include "utils/clock.h"
#include "config_sqp.h"

using namespace util;
using namespace std;
using namespace Eigen;

void StatePlotter::plotTraj(const Eigen::MatrixXd& traj) {
  ScopedStateSave sss(m_ss.get());
  for (int i = 0; i < traj.rows(); i+=SQPConfig::plotDecimation) {
    m_ss->setState(traj.row(i));
    BOOST_FOREACH(EnvironmentObjectPtr& obj, m_scene->env->objects) {
      obj->prePhysics();
      obj->preDraw();
    }
    m_scene->draw();
    printf("step %i. press p to continue\n", i);
    m_scene->idle(true);
  }
}

GripperPlotter::GripperPlotter(RaveRobotObject::Manipulator::Ptr rrom, Scene* scene, int decimation) :
  m_scene(scene), m_osgRoot(scene->env->osg->root.get()), m_rrom(rrom), m_decimation(decimation),
      m_curve(new PlotCurve(3)) {
  m_osgRoot->addChild(m_curve.get());
}

void GripperPlotter::setNumGrippers(int n) {
  if (n != m_grippers.size()) {
    clear();
    m_grippers.resize(n);
    for (int i = 0; i < n; ++i) {
      FakeGripper::Ptr fakeGripper(new FakeGripper(m_rrom));
      m_grippers[i] = fakeGripper;
      m_osgRoot->addChild(fakeGripper->m_node);
    }
  }
}

void GripperPlotter::clear() {

  for (int i = 0; i < m_grippers.size(); ++i) {
    m_osgRoot->removeChild(m_grippers[i]->m_node);
  }
  m_grippers.clear();
}

void GripperPlotter::plotTraj(const MatrixXd& traj) {
  setNumGrippers(traj.rows() / m_decimation);
  vector<btTransform> transforms(m_grippers.size());
  vector<btVector3> origins(m_grippers.size());

  vector<double> curDOFVals = m_rrom->getDOFValues();
  for (int iPlot = 0; iPlot < m_grippers.size(); ++iPlot) {
    m_rrom->setDOFValues(toDoubleVec(traj.row(iPlot * m_decimation)));
    transforms[iPlot] = m_rrom->getTransform();
    origins[iPlot] = transforms[iPlot].getOrigin();
  }
  m_rrom->setDOFValues(curDOFVals);

  m_curve->setPoints(origins);
  for (int iPlot = 0; iPlot < m_grippers.size(); ++iPlot)
    m_grippers[iPlot]->setTransform(transforms[iPlot]);
  m_scene->step(0);
}

GripperPlotter::~GripperPlotter() {
  clear();
  m_osgRoot->removeChild(m_curve);
}

GripperAxesPlotter::GripperAxesPlotter(RaveRobotObject::Manipulator::Ptr manip, int startCol, Environment::Ptr env, float size) :
	m_startCol(startCol), m_manip(manip), m_env(env), m_size(size) {
}
void GripperAxesPlotter::clear() {
	m_handles.reset();
}

GripperAxesPlotter::~GripperAxesPlotter() {
	clear();
}

void GripperAxesPlotter::plotTraj(const Eigen::MatrixXd& traj) {
	ScopedRobotSave(m_manip->robot->robot);
	const vector<int>& armInds = m_manip->manip->GetArmIndices();
	m_manip->robot->robot->SetActiveDOFs(armInds);
	std::vector<EnvironmentObjectPtr> axes(traj.rows());
	for (int i=0; i < traj.rows(); ++i) {
		m_manip->setDOFValues(toDoubleVec(traj.block(i,m_startCol,1,armInds.size()).transpose()));
		axes[i] = PlotAxesPtr(new PlotAxes(m_manip->getTransform(), m_size));
	}
	m_handles.reset(new PlotHandles(axes, m_env));
}

ArmPlotter::ArmPlotter(RaveRobotObject::Manipulator::Ptr rrom, Scene* scene, int decimation) :
  m_syncher(syncherFromArm(rrom)) {
  vector<BulletObject::Ptr> armObjs;
  BOOST_FOREACH(KinBody::LinkPtr link, m_syncher->m_links)
    armObjs.push_back(rrom->robot->associatedObj(link));
  init(rrom, armObjs, scene, decimation);
}

void ArmPlotter::init(RaveRobotObject::Manipulator::Ptr rrom,
    const std::vector<BulletObject::Ptr>& origs, Scene* scene, int decimation) {
  m_rrom = rrom;
  m_scene = scene;
  m_osgRoot = scene->env->osg->root.get();
  m_origs = origs;
  m_decimation = decimation;
  m_curve = new PlotCurve(3);
  m_curve->m_defaultColor = osg::Vec4f(0, 1, 0, 1);
  m_axes.reset(new PlotAxes());
  m_curve->m_defaultColor = osg::Vec4f(0, 1, 0, 1);
  m_scene->env->add(m_axes);
  m_osgRoot->addChild(m_curve.get());
}

ArmPlotter::~ArmPlotter() {
  m_osgRoot->removeChild(m_curve);
  m_scene->env->remove(m_axes);
}

void ArmPlotter::setLength(int nPlots) {
  if (m_fakes.rows() == nPlots) return;
  m_fakes.resize(nPlots, m_origs.size());
  for (int iPlot = 0; iPlot < nPlots; ++iPlot) {
    for (int iObj = 0; iObj < m_origs.size(); ++iObj) {
      FakeObjectCopy::Ptr& fake = m_fakes.at(iPlot, iObj);
      fake.reset(new FakeObjectCopy(m_origs[iObj]));
      fake->makeChildOf(m_osgRoot);
      float frac = (float) iPlot / nPlots;
      fake->setColor(osg::Vec4f(frac, 0, 1 - frac, .35));
    }
  }
}

void ArmPlotter::plotTraj(const MatrixXd& traj) {
  setLength(traj.rows() / m_decimation);

  vector<double> curDOFVals = m_rrom->getDOFValues();
  for (int iPlot = 0; iPlot < m_fakes.rows(); ++iPlot) {
    m_rrom->setDOFValues(toDoubleVec(traj.row(iPlot * m_decimation)));
    m_syncher->updateBullet();
    for (int iObj = 0; iObj < m_origs.size(); ++iObj) {
      m_fakes.at(iPlot, iObj)->setTransform(m_origs[iObj]->rigidBody->getCenterOfMassTransform());
    }
  }

  m_rrom->setDOFValues(toDoubleVec(traj.row(traj.rows() - 1)));
  m_axes->setup(m_rrom->getTransform(), .1 * METERS);

  vector<btVector3> gripperPositions = getGripperPositions(traj, m_rrom);
  m_curve->setPoints(gripperPositions);

  m_rrom->setDOFValues(curDOFVals);
  m_syncher->updateBullet();

  TIC();
  m_scene->step(0);
  LOG_INFO_FMT("draw time: %.3f", TOC());
}

void adjustWorldTransparency(float inc) {
  EnvironmentPtr env = util::getGlobalEnv();
  BOOST_FOREACH(EnvironmentObjectPtr obj, env->objects) {
    obj->adjustTransparency(inc);
  }

}

void interactiveTrajPlot(const MatrixXd& traj, RaveRobotObject::Manipulator::Ptr arm, Scene* scene) {
  RobotBasePtr robot = arm->robot->robot;
  ScopedRobotSave srs(robot);
  BulletRaveSyncherPtr syncher = syncherFromArm(arm);

  vector<int> armInds = arm->manip->GetArmIndices();
  if (armInds.size() == traj.cols()) {
    robot->SetActiveDOFs(arm->manip->GetArmIndices());
  }
  else {
    robot->SetActiveDOFs(armInds, DOF_X | DOF_Y | DOF_RotationAxis, OpenRAVE::RaveVector<double>(0,0,1));
  }

  for (int iStep = 0; iStep < traj.rows(); ++iStep) {
    robot->SetActiveDOFValues(toDoubleVec(traj.row(iStep)));
    arm->robot->updateBullet();
    printf("step %i. press p to continue\n", iStep);
    scene->step(0);
    scene->idle(true);
  }
}

PlotPointsPtr collisions;
PlotLinesPtr escapes;

void clearCollisionPlots() {
  collisions->clear();
  escapes->clear();
}
void plotCollisions(const TrajCartCollInfo& trajCartInfo, double safeDist) {
  if (!collisions) {
    collisions.reset(new PlotPoints(10));
    escapes.reset(new PlotLines(5));
    getGlobalEnv()->add(collisions);
    getGlobalEnv()->add(escapes);
  }

  const osg::Vec4 GREEN(0, 1, 0, 1), YELLOW(1, 1, 0, 1), RED(1, 0, 0, 1);
  osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
  osg::ref_ptr<osg::Vec3Array> escPts = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec3Array> collPts = new osg::Vec3Array;
  for (int iStep = 0; iStep < trajCartInfo.size(); ++iStep) {
//    if (iStep % SQPConfig::plotDecimation != 0) continue;
    for (int iColl = 0; iColl < trajCartInfo[iStep].size(); ++iColl) {
      const LinkCollision& lc = trajCartInfo[iStep][iColl];
      collPts->push_back(toOSGVector(lc.point * METERS));
      escPts->push_back(toOSGVector(lc.point * METERS));
      escPts->push_back(toOSGVector(lc.point * METERS - lc.normal * (lc.dist
          - BulletConfig::linkPadding) * METERS));
      if (lc.dist < 0) colors->push_back(RED);
      else if (lc.dist < safeDist) colors->push_back(YELLOW);
      else colors->push_back(GREEN);
    }
  }
  assert(collPts->size() == colors->size());
  assert(escPts->size() == 2*colors->size());
  collisions->setPoints(collPts, colors);
  escapes->setPoints(escPts, colors);

}

#include <osg/Depth>
void makeFullyTransparent(EnvironmentObject::Ptr obj) {
  osg::Depth* depth = new osg::Depth;
  depth->setWriteMask(false);
  obj->getOSGNode()->getOrCreateStateSet()->setAttributeAndModes(depth, osg::StateAttribute::ON);
}

