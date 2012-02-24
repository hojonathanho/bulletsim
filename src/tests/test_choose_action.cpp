#include "simulation/basescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <fstream>

struct ChooseActionScene : public BaseScene {
  ChooseActionScene();
  BulletSoftObject::Ptr cloth;
};

void saveNodes(btVector3* nodes, const char* fileName) {
  ofstream nodeFile;
  nodeFile.open(fileName);
  int len = sizeof(nodes) / sizeof(nodes[0]);
  for (int i = 0; i < len; i++) {
    nodeFile << nodes[i].x();
    nodeFile << " ";
    nodeFile << nodes[i].y();
    nodeFile << " ";
    nodeFile << nodes[i].z();
    nodeFile << "\n";
  }
  nodeFile.close();
}

btSoftBody* createPatch(btSoftBodyWorldInfo& worldInfo,
			btVector3* x,
			int resx,
			int resy,
			int fixeds,
			bool gendiags) {
#define IDX(_x_,_y_) ((_y_)*rx+(_x_))
  saveNodes(x, "testfile");
  /* Create nodes */
  if(sizeof(x) < 1) return(0);
  const int rx = resx;
  const int ry = resy;
  const int tot = rx*ry;
  btScalar* m = new btScalar[tot];
  for(int i=0;i<tot;++i) {
    m[i]=1;
  }
  btSoftBody* psb=new btSoftBody(&worldInfo,tot,x,m);
  if(fixeds&1) psb->setMass(IDX(0,0),0);
  if(fixeds&2) psb->setMass(IDX(rx-1,0),0);
  if(fixeds&4) psb->setMass(IDX(0,ry-1),0);
  if(fixeds&8) psb->setMass(IDX(rx-1,ry-1),0);
  delete[] x;
  delete[] m;
  /* Create links and faces */
  for(int iy=0;iy<ry;++iy) {
    for(int ix=0;ix<rx;++ix) {
      const int idx=IDX(ix,iy);
      const bool mdx=(ix+1)<rx;
      const bool mdy=(iy+1)<ry;
      if(mdx) psb->appendLink(idx,IDX(ix+1,iy));
      if(mdy) psb->appendLink(idx,IDX(ix,iy+1));
      if(mdx&&mdy) {
	if((ix+iy)&1) {
	  psb->appendFace(IDX(ix,iy),IDX(ix+1,iy),IDX(ix+1,iy+1));
	  psb->appendFace(IDX(ix,iy),IDX(ix+1,iy+1),IDX(ix,iy+1));
	  if(gendiags) {
	    psb->appendLink(IDX(ix,iy),IDX(ix+1,iy+1));
	  }
	}
	else {
	  psb->appendFace(IDX(ix,iy+1),IDX(ix,iy),IDX(ix+1,iy));
	  psb->appendFace(IDX(ix,iy+1),IDX(ix+1,iy),IDX(ix+1,iy+1));
	  if(gendiags) {
	    psb->appendLink(IDX(ix+1,iy),IDX(ix,iy+1));
	  }
	}
      }
    }
  }
  /* Finished */
#undef IDX
  return(psb);
}

btSoftBody* createPatch(btSoftBodyWorldInfo& worldInfo,
			const btVector3& corner00,
			const btVector3& corner10,
			const btVector3& corner01,
			const btVector3& corner11,
			int resx,
			int resy,
			int fixeds,
			bool gendiags) {
  /* Create node array */
  if((resx<2)||(resy<2)) return(0);
  btVector3* x = new btVector3[resx*resy];
  for (int iy=0; iy < resy; ++iy) {
    const btScalar ty=iy/(btScalar)(resy-1);
    const btVector3 py0=lerp(corner00, corner01, ty);
    const btVector3 py1=lerp(corner10, corner11, ty);
    for(int ix=0; ix<resx; ++ix) {
      const btScalar tx=ix/(btScalar)(resx-1);
      x[iy*resx+ix]=lerp(py0,py1,tx);
    }
  }
  return createPatch(worldInfo, x, resx, resy, fixeds, gendiags);
}

ChooseActionScene::ChooseActionScene() {
  // create cloth
  btScalar s = 1;
  btScalar z = 1;
  btVector3* nodes = new btVector3[961];
  for (int i = 0; i < 31; i++) {
    for (int j = 0; j < 31; j++) {
      if (j <= 15) {
	nodes[i+31*j] = btVector3(i*s/30.0, j*s/30.0,z);
      } else {
	nodes[i+31*j] = btVector3(i*s/30.0,s-j*s/30.0,z+.01*(j-15)/30.0);
      }
    }
  }

  btSoftBody* psb = createPatch(env->bullet->softBodyWorldInfo,
				nodes, 31, 31, 0, true);

  /*
  btSoftBody* psb = createPatch(env->bullet->softBodyWorldInfo,
				btVector3(-s,-s,z),
				btVector3(+s,-s,z),
				btVector3(-s,+s,z),
				btVector3(+s,+s,z),
				31, 31,
				0, true);
  */
  psb->getCollisionShape()->setMargin(0.4);
  btSoftBody::Material* pm=psb->appendMaterial();
  pm->m_kLST = 0.4;
  //pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
  psb->generateBendingConstraints(2, pm);
  psb->setTotalMass(150);
  cloth = BulletSoftObject::Ptr(new BulletSoftObject(psb));
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
  while (!scene.viewer.done()) {
    if (step_count % 300 == 0) {
      btSoftBody * const psb = scene.cloth->softBody.get();
      for (int i = 0; i < psb->m_nodes.size(); i++) {
	btVector3 pos = psb->m_nodes[i].m_x;
	btVector3 above = btVector3(pos.x(), pos.y(), pos.z() + 5);
	btVector3 below = btVector3(pos.x(), pos.y(), pos.z() - 5);
	
	cout << i;
	cout << " ";
	cout << rayTest(psb, above, below);
	cout << ", ";
	
      }
      cout << "\n";
    }
    step_count++;
    scene.step(.01);
  }
  return 0;
}
