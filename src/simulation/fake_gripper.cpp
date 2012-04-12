#include "simulation/fake_gripper.h"
#include "boost/foreach.hpp"
#include "utils/conversions.h"
using namespace std;
using namespace OpenRAVE;
using boost::shared_ptr;
using osg::ref_ptr;


void FakeGripper::setTransform(const btTransform& tf) {
  btTransform tf_manip = m_manip->getTransform();
  m_transform->setMatrix(toOsgMatrix(tf * tf_manip.inverse()));
}

FakeGripper::FakeGripper(RaveRobotObject::Manipulator::Ptr manip) : m_manip(manip) {
  m_node = new osg::Group;
  m_transform = new osg::MatrixTransform(osg::Matrixf::identity());
  vector<KinBody::LinkPtr> links; manip->manip->GetChildLinks(links);
  RaveRobotObject* robot = m_manip->robot;
  m_node->addChild(m_transform.get());

  BOOST_FOREACH(KinBody::LinkPtr link, links) {
    BulletObject::Ptr obj = robot->associatedObj(link);
    if (obj) {
      ref_ptr<osg::Node> objnode = obj->transform;
      m_transform->addChild(objnode);
    }
  }

  setTransform(btTransform::getIdentity());

}

void TelekineticGripper::prePhysics() {
  btTransform tf_manip = m_manip->getTransform();
  btTransform tf_obj;
  for (int i=0; i < m_origs.size(); i++) {
    m_origs[i]->motionState->getWorldTransform(tf_obj);
    children[i]->motionState->setKinematicPos(m_tf * tf_manip.inverse()*tf_obj);
  }

}

btTransform TelekineticGripper::getLinkTransform(KinBody::LinkPtr link) const {
  btTransform tf_manip = m_manip->getTransform();
  btTransform tf_obj = m_manip->robot->getLinkTransform(link);
  return m_tf * tf_manip.inverse()*tf_obj;
}

BulletObject::Ptr TelekineticGripper::getLinkRigidBody(KinBody::LinkPtr link) const {
  return children[m_linkToChildMap.find(link)->second];
}

void TelekineticGripper::setTransform(const btTransform& tf) {m_tf = tf;}

btTransform TelekineticGripper::getTransform() const {
  return m_tf;
}

TelekineticGripper::TelekineticGripper(RaveRobotObject::Manipulator::Ptr manip) : m_manip(manip) {
  m_tf.setIdentity();

  vector<KinBody::LinkPtr> links; manip->manip->GetChildLinks(links);
  RaveRobotObject* robot = m_manip->robot;

  BOOST_FOREACH(KinBody::LinkPtr link, links) {
    BulletObject::Ptr obj = robot->associatedObj(link);
    if (obj) {
      children.push_back(BulletObject::Ptr(new BulletObject(*obj)));
      m_origs.push_back(obj);
      m_linkToChildMap[link] = children.size() - 1;
    }
  }
}

EnvironmentObject::Ptr TelekineticGripper::copy(Fork &f) const {
  Ptr o(new TelekineticGripper());
  o->m_tf = m_tf;
  CompoundObject::internalCopy(o, f);
  return o;
}

void TelekineticGripper::postCopy(EnvironmentObject::Ptr copy, Fork &f) const {
  Ptr o = boost::static_pointer_cast<TelekineticGripper>(copy);

  RaveRobotObject::Ptr forkRobot = boost::static_pointer_cast<RaveRobotObject>(f.forkOf(m_manip->robot));
  BOOST_ASSERT(forkRobot);
  o->m_manip = forkRobot->getManipByIndex(m_manip->index);

  vector<KinBody::LinkPtr> links; o->m_manip->manip->GetChildLinks(links);
  int count = 0;
  BOOST_FOREACH(KinBody::LinkPtr link, links) {
    BulletObject::Ptr obj = forkRobot->associatedObj(link);
    if (obj) {
      o->m_origs.push_back(obj);
      o->m_linkToChildMap[link] = count++;
    }
  }
  BOOST_ASSERT(o->m_origs.size() == m_origs.size());
}
