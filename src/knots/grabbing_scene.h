#pragma once
#include "simulation/simplescene.h"
#include "robots/grabbing.h"
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

};
