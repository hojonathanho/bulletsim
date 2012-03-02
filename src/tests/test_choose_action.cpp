#include "simulation/basescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/bullet_io.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <fstream>

struct ChooseActionScene : public BaseScene {
  ChooseActionScene();
  BulletSoftObject::Ptr cloth;
};

ChooseActionScene::ChooseActionScene() {
  // create cloth
  btScalar s = 1;
  btScalar z = .1;
  btSoftBody* psb = btSoftBodyHelpers::CreatePatch(env->bullet->softBodyWorldInfo,
						   btVector3(-s,-s,z),
						   btVector3(+s,-s,z),
						   btVector3(-s,+s,z),
						   btVector3(+s,+s,z),
						   31, 31,
						   0, true);
  /*
  psb->m_cfg.piterations = 2;
  psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
    | btSoftBody::fCollision::CL_RS
    | btSoftBody::fCollision::CL_SELF;
  psb->m_cfg.kDF = 1.0;
  psb->getCollisionShape()->setMargin(0.05);
  btSoftBody::Material *pm = psb->appendMaterial();
  pm->m_kLST = 0.1;
  psb->generateBendingConstraints(2, pm);
  psb->randomizeConstraints();
  psb->setTotalMass(1, true);
  psb->generateClusters(0);
  */
  psb->getCollisionShape()->setMargin(0.4);
  btSoftBody::Material* pm=psb->appendMaterial();
  pm->m_kLST = 0.4;
  //pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
  psb->generateBendingConstraints(2, pm);
  psb->setTotalMass(150);
  
  cloth = BulletSoftObject::Ptr(new BulletSoftObject(psb));
  //cloth = BulletSoftObject::Ptr(new BulletSoftObject(loadSoftBody(env->bullet->softBodyWorldInfo, "testfile.txt")));
  
  //cout << "saving softbody\n";
  //saveSoftBody(psb, "testfile.txt");
  env->add(cloth);
}

btScalar rayFromToTriangle(const btVector3& rayFrom,
			   const btVector3& rayTo,
			   const btVector3& rayNormalizedDirection,
			   const btVector3& a,
			   const btVector3& b,
			   const btVector3& c,
			   btScalar maxt) {
  static const btScalar	ceps=-SIMD_EPSILON*10;
  static const btScalar	teps=SIMD_EPSILON*10;

  const btVector3 n=btCross(b-a,c-a);
  const btScalar d=btDot(a,n);
  const btScalar den=btDot(rayNormalizedDirection,n);
  if(!btFuzzyZero(den)) {
    const btScalar num=btDot(rayFrom,n)-d;
    const btScalar t=-num/den;
    if((t>teps)&&(t<maxt)) {
      const btVector3 hit=rayFrom+rayNormalizedDirection*t;
      if((btDot(n,btCross(a-hit,b-hit))>ceps)
	 && (btDot(n,btCross(b-hit,c-hit))>ceps)
	 && (btDot(n,btCross(c-hit,a-hit))>ceps)) {
	return(t);
      }
    }
  }
  return(-1);
}

int rayTest(btSoftBody* const psb, const btVector3& rayFrom, const btVector3& rayTo) {
  int count = 0;
  btVector3 dir = rayTo - rayFrom;
  vector<btSoftBody::Node *> nodes;
  for(int i = 0,ni = psb->m_faces.size(); i < ni; ++i) {
    const btSoftBody::Face& f = psb->m_faces[i];
    const btScalar t = rayFromToTriangle(rayFrom,
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
  /*
  BulletInstance::Ptr bullet2;
  OSGInstance::Ptr osg2;
  Fork::Ptr fork;
  */
  while (!scene.viewer.done()) {
    if (step_count == 300) {
      btSoftBody * const psb = scene.cloth->softBody.get();
      //cout << "saving softbody\n";
      //saveSoftBody(psb, "testfile.txt");

      /*
      bullet2.reset(new BulletInstance);
      bullet2->setGravity(BulletConfig::gravity);
      osg2.reset(new OSGInstance);
      scene.osg->root->addChild(osg2->root.get());

      fork.reset(new Fork(scene.env, bullet2, osg2));
      //scene.registerFork(fork);
      */
      for (int i = 0; i < psb->m_nodes.size(); i++) {
	const btVector3& pos = psb->m_nodes[i].m_x;
	const btVector3& above = btVector3(pos.x(), pos.y(), pos.z() + 1);
	const btVector3& below = btVector3(pos.x(), pos.y(), pos.z() + .0001);
	int layersAbove = rayTest(psb, above, below);
	cout << i;
	cout << " ";
	cout << layersAbove;
	cout << ", ";
	if (layersAbove == 0) {
	  SphereObject::Ptr sphere(new SphereObject(0, 0.01, btTransform(btQuaternion(0, 0, 0, 1), above)));
	  scene.env->add(sphere);
	} else {
	  BoxObject::Ptr box(new BoxObject(0, btVector3(0.01, 0.01, 0.01), btTransform(btQuaternion(0, 0, 0, 1), above)));
	  scene.env->add(box);
	}
      }
      cout << "\n";
    }
    step_count++;
    scene.step(.01);
  }
  return 0;
}
