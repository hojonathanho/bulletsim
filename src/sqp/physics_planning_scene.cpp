#include "physics_planning_scene.h"
#include "simulation/config_bullet.h"
#include "simulation/util.h"

PhysicsPlanningScene::PhysicsPlanningScene(EnvironmentBasePtr rave_) :
  Scene(rave_),
  m_planScene(new Scene(rave_)),
  m_planVisible(false) {
  
  float padding = BulletConfig::linkPadding;  
  BulletConfig::linkPadding = 0;
  LoadFromRave(env, rave);
  
  BulletConfig::linkPadding = padding;
  LoadFromRave(m_planScene->env, m_planScene->rave);
  
  util::setGlobalEnv(env);
  util::setGlobalScene(this);

  addVoidKeyCallback('b',boost::bind(&PhysicsPlanningScene::toggleVisible, this), "toggle visible");
}


void PhysicsPlanningScene::step(float dt, int maxsteps, float internaldt) {
  Scene::step(dt, maxsteps, internaldt);
  m_planScene->step(0, 0, 0);
}

typedef map<KinBodyPtr, RaveObject*>::value_type KB_RO;



void PhysicsPlanningScene::toggleVisible() {
  if (m_planVisible) {
    osg->root->removeChild(m_planScene->osg->root);
    m_planVisible = false;
  }
  else {
    osg->root->addChild(m_planScene->osg->root);
    m_planVisible = true;
    BOOST_FOREACH(KB_RO kb_ro, m_planScene->rave->rave2bulletsim) {
      kb_ro.second->updateBullet();
    }
    m_planScene->step(0);
  } 
}

void PhysicsPlanningScene::updateRaveFromBullet() {
  BOOST_FOREACH(KB_RO kb_ro, rave->rave2bulletsim) {
    if (!kb_ro.first->IsRobot())
      kb_ro.first->SetTransform(util::toRaveTransform(kb_ro.second->children[0]->rigidBody->getCenterOfMassTransform(), 1 / METERS));
  }
}

void PhysicsPlanningScene::updatePlanningScene() {
  BOOST_FOREACH(KB_RO kb_ro, m_planScene->rave->rave2bulletsim) {
    kb_ro.second->updateBullet();
  }
}

