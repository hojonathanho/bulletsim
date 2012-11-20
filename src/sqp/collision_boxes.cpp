#include "collision_boxes.h"
#include <osgbCollision/CollisionShapes.h>
#include "simulation/set_colors_visitor.h"

void CollisionBoxes::construct(const std::vector<btVector3>& centers, const std::vector<btVector3> extents, const std::vector<osg::Vec4f>& colors) {

  assert(centers.size() == colors.size());
  assert(centers.size() == extents.size());
  bool useColors = colors.size() > 0;
  if (useColors) assert(centers.size() == colors.size());
    
  m_compound.reset(new btCompoundShape());  
  m_root = new osg::Group();
  
  int nBoxes = centers.size();
  m_shapes.reserve(nBoxes);
  m_nodes.reserve(nBoxes);
  
  for (int i=0; i < nBoxes; ++i) {
//    CollisionShapePtr shapePtr(new btBoxShape(extents[i]));
  	CollisionShapePtr shapePtr(new btSphereShape(extents[0].x()));
    btTransform boxTrans(btQuaternion::getIdentity(), centers[i]);
    NodePtr nodePtr = osg::ref_ptr<osg::Node>(osgbCollision::osgNodeFromBtCollisionShape(shapePtr.get(), boxTrans));
    m_shapes.push_back(shapePtr);
    m_nodes.push_back(nodePtr);
    m_compound->addChildShape(boxTrans, shapePtr.get());
    m_root->addChild(nodePtr.get());
    if (useColors) {
      SetColorsVisitor visitor(colors[i]);
      nodePtr->accept(visitor);      
    }
  }
    
  m_motionState.reset(new btDefaultMotionState(btTransform::getIdentity()));
	btRigidBody::btRigidBodyConstructionInfo rbInfo(0 /*mass*/,m_motionState.get(),m_compound.get(), btVector3(0,0,0) /*local intertia*/);
  m_rigidBody.reset(new btRigidBody(rbInfo));
  
}

void CollisionBoxes::init() {
  getEnvironment()->bullet->dynamicsWorld->addRigidBody(m_rigidBody.get());
  getEnvironment()->osg->root->addChild(m_root.get());  
}

void CollisionBoxes::destroy() {
  getEnvironment()->bullet->dynamicsWorld->removeRigidBody(m_rigidBody.get());
  getEnvironment()->osg->root->removeChild(m_root.get());  
}



