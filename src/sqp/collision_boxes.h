#pragma once
#include "simulation/environment.h"


class CollisionBoxes : public EnvironmentObject {

  typedef boost::shared_ptr<btRigidBody> RigidBodyPtr;
  typedef boost::shared_ptr<btCollisionShape> CollisionShapePtr;
  typedef osg::ref_ptr<osg::Node> NodePtr;

  std::vector<NodePtr> m_nodes;
  std::vector<CollisionShapePtr> m_shapes;

  RigidBodyPtr m_rigidBody;
  boost::shared_ptr<btCompoundShape> m_compound;
  osg::ref_ptr<osg::Group> m_root;
  boost::shared_ptr<btDefaultMotionState> m_motionState;

  void construct(const std::vector<btVector3>& centers, const std::vector<btVector3> extents, const std::vector<osg::Vec4f>& colors);
  
public:
  typedef boost::shared_ptr<CollisionBoxes> Ptr;
  
  CollisionBoxes(const std::vector<btVector3>& centers, const std::vector<btVector3> extents, const std::vector<osg::Vec4f>& colors) {
    construct(centers, extents, colors);
  }
  void init();
  void destroy();
  virtual EnvironmentObject::Ptr copy(Fork &f) const {assert(0);}
};
