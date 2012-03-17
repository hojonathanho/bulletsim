#pragma once
#include "simulation/simplescene.h"
#include "robots/grabbing.h"
#include <boost/bind.hpp>
using boost::shared_ptr;

struct GrabbingScene : public Scene {
public:
  shared_ptr<PR2Manager> pr2m;
  shared_ptr<MonitorForGrabbing> m_lMonitor;
  shared_ptr<MonitorForGrabbing> m_rMonitor;

  GrabbingScene() {
    pr2m.reset(new PR2Manager(*this));
    m_lMonitor.reset(new MonitorForGrabbing(pr2m->pr2Left, env->bullet->dynamicsWorld));
    m_rMonitor.reset(new MonitorForGrabbing(pr2m->pr2Right, env->bullet->dynamicsWorld));
    pr2m->setHapticCb(hapticLeft0Hold, boost::bind(&GrabbingScene::closeLeft, this));
    pr2m->setHapticCb(hapticLeft1Hold, boost::bind(&GrabbingScene::openLeft, this));
    pr2m->setHapticCb(hapticRight0Hold, boost::bind(&GrabbingScene::closeRight, this));
    pr2m->setHapticCb(hapticRight1Hold, boost::bind(&GrabbingScene::openRight, this));
  }

  void step(float dt) {
    m_lMonitor->update();
    m_rMonitor->update();
    Scene::step(dt);
  }


  void setGrabBodies(vector<BulletObject::Ptr> bodies) {
    m_lMonitor->setBodies(bodies);
    m_rMonitor->setBodies(bodies);
  }

  void closeLeft() {
    pr2m->pr2Left->setGripperAngle(pr2m->pr2Left->getGripperAngle() - .02);
  }
  void openLeft() {
    pr2m->pr2Left->setGripperAngle(pr2m->pr2Left->getGripperAngle() + .02);
  }
  void closeRight() {
    pr2m->pr2Right->setGripperAngle(pr2m->pr2Right->getGripperAngle() - .02);
  }
  void openRight() {
    pr2m->pr2Right->setGripperAngle(pr2m->pr2Right->getGripperAngle() + .02);
  }

};
