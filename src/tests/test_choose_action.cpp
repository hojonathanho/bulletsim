#include "simulation/basescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

struct ChooseActionScene : public BaseScene {
  ChooseActionScene();
  void step(float dt, int maxsteps, float internaldt);
  int step_count;
  BulletSoftObject::Ptr cloth;
};

ChooseActionScene::ChooseActionScene() {
  step_count = 0;
  // create cloth
  btScalar s = 1;
  btScalar z = .1;
  btSoftBody* psb =
    btSoftBodyHelpers::CreatePatch(env->bullet->softBodyWorldInfo,
				   btVector3(-s,-s,z),
				   btVector3(+s,-s,z),
				   btVector3(-s,+s,z),
				   btVector3(+s,+s,z),
				   31, 31,
				   0, true);
  for (int i = 0; i < psb->m_nodes.size(); i++) {
    btVector3 pos = psb->m_nodes[i].m_x;
    cout << pos.x();
    cout << " ";
    cout << pos.y();
    cout << " ";
    cout << pos.z();
    cout << "\n";
  }
  psb->getCollisionShape()->setMargin(0.4);
  btSoftBody::Material* pm=psb->appendMaterial();
  pm->m_kLST = 0.4;
  //pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
  psb->generateBendingConstraints(2, pm);
  psb->setTotalMass(150);
  cloth = BulletSoftObject::Ptr(new BulletSoftObject(psb));
  env->add(cloth);
}

void ChooseActionScene::step(float dt, int maxsteps, float internaldt) {
  if (step_count % 300 == 0) {
    btSoftBody * const psb = cloth->softBody.get();
    btVector3 pos = psb->m_nodes[23].m_x;
    btVector3 above = btVector3(pos.x(), pos.y(), pos.z() + 5);
    btVector3 below = btVector3(pos.x(), pos.y(), pos.z() - 5);
    btSoftBody::sRayCast s;
    s.body = psb;
    s.fraction = 1.f;
    s.feature = btSoftBody::eFeature::None;
    s.index = -1;
    cout << psb->rayTest(above, below, s.fraction, s.feature, s.index, true);
    cout << "\n";
    cout << s.body;
    cout << "\n";
    cout << psb;
    cout << "\n";
    cout << s.index;
    cout << "\n";
    cout << s.feature;
    cout << "\n";
    cout << s.fraction;
    cout << "\n";
    /*for (int i = 0; i < psb->m_nodes.size(); i++) {
      btVector3 pos = psb->m_nodes[i].m_x;
      cout << pos.x();
      cout << " ";
      cout << pos.y();
      cout << " ";
      cout << pos.z();
      cout << "\n";
      }*/
  }
  step_count++;
  BaseScene::step(dt, maxsteps, internaldt);
}

int main(int argc, char *argv[]) {
  GeneralConfig::scale = 10.;    

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);
  
  ChooseActionScene scene;
  
  scene.startViewer();
  scene.startLoop();
  return 0;
}
