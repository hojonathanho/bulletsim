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
public:
  typedef boost::shared_ptr<TelekineticGripper> Ptr;
  RaveRobotObject::Manipulator::Ptr m_manip;
  vector<BulletObject::Ptr> m_origs;
  btTransform m_tf;

  void prePhysics();
  void setTransform(const btTransform& tf);

  TelekineticGripper(RaveRobotObject::Manipulator::Ptr manip);
};