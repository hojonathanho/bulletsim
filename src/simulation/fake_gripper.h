#pragma once
#include "simulation/openravesupport.h"

class FakeGripper {
public:
  typedef boost::shared_ptr<FakeGripper> Ptr;
  RaveRobotObject::Manipulator::Ptr m_manip;
  osg::ref_ptr<osg::Group> m_node;
  osg::ref_ptr<osg::MatrixTransform> m_transform;

  void setTransform(const btTransform& tf);

  FakeGripper(RaveRobotObject::Manipulator::Ptr manip);


};

class TelekineticGripper : public CompoundObject<BulletObject> {
  map<KinBody::LinkPtr, int> m_linkToChildMap;
  TelekineticGripper() { }
public:
  typedef boost::shared_ptr<TelekineticGripper> Ptr;
  RaveRobotObject::Manipulator::Ptr m_manip;
  vector<BulletObject::Ptr> m_origs;
  btTransform m_tf;

  void prePhysics();
  void setTransform(const btTransform& tf);
  btTransform getTransform() const;
  btTransform getLinkTransform(KinBody::LinkPtr link) const;
  BulletObject::Ptr getLinkRigidBody(KinBody::LinkPtr link) const;

  TelekineticGripper(RaveRobotObject::Manipulator::Ptr manip);

  EnvironmentObject::Ptr copy(Fork &f) const;
  void postCopy(EnvironmentObject::Ptr copy, Fork &f) const;
};
