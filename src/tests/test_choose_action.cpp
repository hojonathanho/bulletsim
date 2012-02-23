#include "simulation/basescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBody.cpp>

struct ChooseActionScene : public BaseScene {
  ChooseActionScene();
  BulletSoftObject::Ptr cloth;
};

ChooseActionScene::ChooseActionScene() {
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

int rayTest(btSoftBody* const psb, const btVector3& rayFrom, const btVector3& rayTo) {
  int count = 0;
  btVector3 dir = rayTo - rayFrom;
  vector<btSoftBody::Node *> nodes;
  for(int i = 0,ni = psb->m_faces.size(); i < ni; ++i) {
    const btSoftBody::Face& f = psb->m_faces[i];
    const btScalar t = btSoftBody::RayFromToCaster::rayFromToTriangle(rayFrom,
							       rayTo,
							       dir,
							       f.m_n[0]->m_x,
							       f.m_n[1]->m_x,
							       f.m_n[2]->m_x,
							       1.f);
    if(t > 0) {
      bool nodeFound = false;
      for (int j = 0; j < nodes.size(); ++j) {
	if (f.m_n[0] == nodes[j] || f.m_n[1] == nodes[j] || f.m_n[2] == nodes[j]) {
	  cout << "common node found\n";
	  nodeFound = true;
	  break;
	}
      }
      if (!nodeFound) {
	nodes.push_back(f.m_n[0]);
	nodes.push_back(f.m_n[1]);
	nodes.push_back(f.m_n[2]);
	++count;
      }
    }
  }
  return count;
}

int main(int argc, char *argv[]) {
  GeneralConfig::scale = 10.;    

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);
  
  ChooseActionScene scene;
  
  scene.startViewer();
  int step_count = 0;
  while (!scene.viewer.done()) {
    if (step_count % 300 == 0) {
      btSoftBody * const psb = scene.cloth->softBody.get();
      btVector3 pos = psb->m_nodes[960].m_x;
      btVector3 above = btVector3(pos.x(), pos.y(), pos.z() + 5);
      btVector3 below = btVector3(pos.x(), pos.y(), pos.z() - 5);
      btSoftBody::sRayCast s;
      s.body = psb;
      s.fraction = 1.f;
      s.feature = btSoftBody::eFeature::None;
      s.index = -1;
      cout << rayTest(psb, above, below);
      cout << "\n";
      /*
      cout << psb->rayTest(above, below, s.fraction, s.feature, s.index, false);
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
      cout << "\n";*/
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
    scene.step(.01);
  }
  return 0;
}
