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

class FakeObjectCopy {
public:
	typedef boost::shared_ptr<FakeObjectCopy> Ptr;
	BulletObject::Ptr m_orig;
	osg::ref_ptr<osg::MatrixTransform> m_transform;
	osg::Group* m_parent;
	FakeObjectCopy(BulletObject::Ptr orig);
	~FakeObjectCopy();
	void setTransform(const btTransform& tf);
	void makeChildOf(osg::Group* parent);
	void setColor(const osg::Vec4f& color);
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
