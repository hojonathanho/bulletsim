#include "simulation/basescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

struct ChooseActionScene : public BaseScene {
  ChooseActionScene();
  btSoftBody* psb;
};

ChooseActionScene::ChooseActionScene() {
  // create cloth
  btScalar s = 5;
  btScalar z = 30;
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
  env->add(BulletSoftObject::Ptr(new BulletSoftObject(psb)));  
}

void ChooseActionScene::step(float dt, int maxsteps, float internaldt) {
  // log stuff
  // step parent
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
