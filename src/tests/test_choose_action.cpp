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

ostream &operator<<(ostream &stream, const btSoftBody::Material* mat) {
  stream << mat->m_flags << " ";
  stream << mat->m_kAST << " ";
  stream << mat->m_kLST << " ";
  stream << mat->m_kVST;
  return stream;
}

ostream &operator<<(ostream &stream, const btSoftBody::Node* node) {
  stream << node->m_x << " ";
  stream << node->m_area << " ";
  stream << node->m_battach << " ";
  stream << node->m_f << " ";
  stream << node->m_im << " ";
  stream << node->m_n << " ";
  stream << node->m_q << " ";
  stream << node->m_v << " ";
  stream << node->m_material;
  return stream;
}

void saveSoftBody(const btSoftBody* orig, const char* fileName) {  
  int i, j;
  ofstream saveFile;
  saveFile.open(fileName);

  // materials
  saveFile << orig->m_materials.size() << endl;
  for (i = 0;i < orig->m_materials.size(); i++) {
    const btSoftBody::Material *mat = orig->m_materials[i];
    saveFile << mat << endl;
  }

  // nodes
  saveFile << orig->m_nodes.size() << endl;
  for (i = 0; i < orig->m_nodes.size(); i++) {
    const btSoftBody::Node *node = &orig->m_nodes[i];
    saveFile << node << endl;
  }
  
  // links
  saveFile << orig->m_links.size() << endl;
  for (i = 0; i < orig->m_links.size(); i++) {
    const btSoftBody::Link *link = &orig->m_links[i];
    saveFile << link->m_material << endl;
    saveFile << link->m_n[0] << endl;
    saveFile << link->m_n[1] << endl;
    saveFile << link->m_bbending << " ";
    saveFile << link->m_rl << endl;
  }

  // faces
  saveFile << orig->m_faces.size();
  for (i = 0; i < orig->m_faces.size(); i++) {
    const btSoftBody::Face *face = &orig->m_faces[i];
    saveFile << face->m_material << endl;
    saveFile << face->m_n[0] << endl;
    saveFile << face->m_n[1] << endl;
    saveFile << face->m_n[2] << endl;
    saveFile << face->m_normal << " ";
    saveFile << face->m_ra << endl;
  }

  // pose
  saveFile << orig->m_pose.m_bvolume << " ";
  saveFile << orig->m_pose.m_bframe << " ";
  saveFile << orig->m_pose.m_volume << endl;
  saveFile << orig->m_pose.m_pos.size() << endl;
  for (i = 0; i < orig->m_pose.m_pos.size(); i++) {
    saveFile << orig->m_pose.m_pos[i] << endl;
  }
  saveFile << orig->m_pose.m_wgh.size() << endl;
  for (i = 0; i < orig->m_pose.m_wgh.size(); i++) {
    saveFile << orig->m_pose.m_wgh[i] << endl;
  }
  saveFile << orig->m_pose.m_com << " ";
  saveFile << orig->m_pose.m_rot << " ";
  saveFile << orig->m_pose.m_scl << " ";
  saveFile << orig->m_pose.m_aqq << endl;

  // config
  saveFile << orig->m_cfg.aeromodel << " ";
  saveFile << orig->m_cfg.kVCF << " ";
  saveFile << orig->m_cfg.kDP << " ";
  saveFile << orig->m_cfg.kDG << " ";
  saveFile << orig->m_cfg.kLF << " ";
  saveFile << orig->m_cfg.kPR << " ";
  saveFile << orig->m_cfg.kVC << " ";
  saveFile << orig->m_cfg.kDF << " ";
  saveFile << orig->m_cfg.kMT << " ";
  saveFile << orig->m_cfg.kCHR << " ";
  saveFile << orig->m_cfg.kKHR << " ";
  saveFile << orig->m_cfg.kSHR << " ";
  saveFile << orig->m_cfg.kAHR << " ";
  saveFile << orig->m_cfg.kSRHR_CL << " ";
  saveFile << orig->m_cfg.kSKHR_CL << " ";
  saveFile << orig->m_cfg.kSSHR_CL << " ";
  saveFile << orig->m_cfg.kSR_SPLT_CL << " ";
  saveFile << orig->m_cfg.kSK_SPLT_CL << " ";
  saveFile << orig->m_cfg.kSS_SPLT_CL << " ";
  saveFile << orig->m_cfg.maxvolume << " ";
  saveFile << orig->m_cfg.timescale << " ";
  saveFile << orig->m_cfg.viterations << " ";
  saveFile << orig->m_cfg.piterations << " ";
  saveFile << orig->m_cfg.diterations << " ";
  saveFile << orig->m_cfg.citerations << " ";
  saveFile << orig->m_cfg.collisions << endl;
  saveFile << orig->m_cfg.m_vsequence.size() << endl;
  for (i = 0; i < orig->m_cfg.m_vsequence.size(); i++) {
    saveFile << orig->m_cfg.m_vsequence[i] << endl;
  }
  saveFile << orig->m_cfg.m_psequence.size() << endl;
  for (i = 0; i < orig->m_cfg.m_psequence.size(); i++) {
    saveFile << orig->m_cfg.m_psequence[i] << endl;
  }
  saveFile << orig->m_cfg.m_dsequence.size() << endl;
  for (i = 0; i < orig->m_cfg.m_dsequence.size(); i++) {
    saveFile << orig->m_cfg.m_dsequence[i] << endl;
  }
  saveFile << orig->getCollisionShape()->getMargin() << endl;

  // solver state
  saveFile << orig->m_sst.isdt << " ";
  saveFile << orig->m_sst.radmrg << " ";
  saveFile << orig->m_sst.sdt << " ";
  saveFile << orig->m_sst.updmrg << " ";
  saveFile << orig->m_sst.velmrg << endl;
  
  // clusters
  saveFile << orig->m_clusters.size() << endl;
  for (i = 0; i < orig->m_clusters.size(); i++) {
    btSoftBody::Cluster *cl = orig->m_clusters[i];
    saveFile << cl->m_nodes.size() << endl;
    for (j = 0; j < cl->m_nodes.size(); j++)
      saveFile << cl->m_nodes[j] << endl;
    saveFile << cl->m_masses.size() << endl;
    for (j = 0; j < cl->m_masses.size(); j++)
      saveFile << cl->m_masses[j] << endl;
    saveFile << cl->m_framerefs.size() << endl;
    for (j = 0; j < cl->m_framerefs.size(); j++)
      saveFile << cl->m_framerefs[j] << endl;
    saveFile << cl->m_framexform << " ";
    saveFile << cl->m_idmass << " ";
    saveFile << cl->m_imass << " ";
    saveFile << cl->m_locii << " ";
    saveFile << cl->m_invwi << " ";
    saveFile << cl->m_com << " ";
    saveFile << cl->m_vimpulses[0] << " ";
    saveFile << cl->m_vimpulses[1] << " ";
    saveFile << cl->m_dimpulses[0] << " ";
    saveFile << cl->m_dimpulses[1] << " ";
    saveFile << cl->m_nvimpulses << " ";
    saveFile << cl->m_ndimpulses << " ";
    saveFile << cl->m_lv << " ";
    saveFile << cl->m_av << " ";
    saveFile << cl->m_ndamping << " ";
    saveFile << cl->m_ldamping << " ";
    saveFile << cl->m_adamping << " ";
    saveFile << cl->m_matching << " ";
    saveFile << cl->m_maxSelfCollisionImpulse << " ";
    saveFile << cl->m_selfCollisionImpulseFactor << " ";
    saveFile << cl->m_containsAnchor << " ";
    saveFile << cl->m_collide << " ";
    saveFile << cl->m_clusterIndex << endl;
  }

  // cluster connectivity
  saveFile << orig->m_clusterConnectivity.size() << endl;
  for (i = 0; i < orig->m_clusterConnectivity.size(); i++) {
    saveFile << orig->m_clusterConnectivity[i] << endl;
  }

  saveFile.close();
}

btSoftBody* loadSoftBody(const char* fileName) {
  ifstream loadFile;
  loadFile.open(fileName);
  loadFile.close();
}

btSoftBody* createPatch(btSoftBodyWorldInfo& worldInfo,
			btVector3* x,
			int resx,
			int resy,
			int fixeds,
			bool gendiags) {
#define IDX(_x_,_y_) ((_y_)*rx+(_x_))
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
  /*
  btSoftBody* psb = createPatch(env->bullet->softBodyWorldInfo,
				nodes, 31, 31, 0, true);

  */
  btSoftBody* psb = createPatch(env->bullet->softBodyWorldInfo,
				btVector3(-s,-s,z),
				btVector3(+s,-s,z),
				btVector3(-s,+s,z),
				btVector3(+s,+s,z),
				31, 31,
				0, true);

  psb->getCollisionShape()->setMargin(0.4);
  btSoftBody::Material* pm=psb->appendMaterial();
  pm->m_kLST = 0.4;
  //pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
  psb->generateBendingConstraints(2, pm);
  psb->setTotalMass(150);
  saveSoftBody(psb, "testfile.txt");
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
	/*
	cout << i;
	cout << " ";
	cout << rayTest(psb, above, below);
	cout << ", ";
	*/
      }
      //cout << "\n";
    }
    step_count++;
    scene.step(.01);
  }
  return 0;
}
