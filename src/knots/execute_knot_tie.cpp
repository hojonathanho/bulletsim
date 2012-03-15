#include "simulation/simplescene.h"
#include "robots/grabbing.h"

class GrabbingScene : public Scene {
  MonitorForGrabbing m_lMonitor;
  MonitorForGrabbing m_rMonitor;

  GrabbingScene() :
    m_lMonitor(pr2m.pr2->robot->GetManipulators()[5], scene.env->bullet->dynamicsWorld),
    m_rMonitor(pr2m.pr2->robot->GetManipulators()[7], scene.env->bullet->dynamicsWorld)
  {}

  void step(float dt, int maxsteps, float internaldt) {
    m_lMonitor.update();
    m_rMonitor.update();
    Scene::step(dt, maxsteps, internaldt);
  }

  void setGrabBodies(vector<BulletObject::Ptr> bodies) {
    m_lMonitor.setBodies(bodies);
    m_rMonitor.seBodies(bodies);
  }

};

int main(int argc, char* argv[]) {
  GrabbingScene scene;
  PR2Manager pr2m(scene);

  scene.setGrabBodies(rope->children);

  while (true) {

    scene.step(DT);

  }

}
