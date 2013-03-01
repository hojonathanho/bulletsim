#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/tetgen_helpers.h"
#include "simulation/rope.h"
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

const string data_dir = EXPAND(BULLETSIM_DATA_DIR)"/clothing/";
#define DATA(str) (data_dir+str).c_str()

//
// TetraFile
//
void	initTetraFile(Scene& scene)
{
	//Create your psb
	btSoftBody* psb=CreateFromTetGenFile(*scene.bullet->softBodyWorldInfo,
			DATA("shirt.1.ele"),
			DATA("shirt.1.face"),
			DATA("shirt.1.node"),
			false,true,true);

	scene.env->add(BulletSoftObject::Ptr(new BulletSoftObject(psb)));

	psb->scale(btVector3(1,1,1));
	psb->translate(btVector3(0,0,5));
	psb->setVolumeMass(100);

	///fix one vertex
	//psb->setMass(0,0);
	//psb->setMass(10,0);
	//psb->setMass(20,0);
	psb->m_cfg.piterations=1;
	//psb->generateClusters(128);
	psb->generateClusters(16);
	//psb->getCollisionShape()->setMargin(0.5);

	psb->getCollisionShape()->setMargin(0.01);
	psb->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS+	btSoftBody::fCollision::CL_RS
		//+ btSoftBody::fCollision::CL_SELF
		;
	psb->m_materials[0]->m_kLST=0.08;
  psb->m_materials[0]->m_kAST		=	0.1;
  psb->m_materials[0]->m_kVST		=	0.1;

	//psb->generateBendingConstraints(2);

	psb->m_cfg.kDF=1.0;
}

btSoftBody* makeTetraPrism(Scene& scene, vector<btVector3> corners_base, btVector3 polygon_translation) {
	//Create your psb
	btSoftBody* psb=CreatePrism(*scene.bullet->softBodyWorldInfo, corners_base, polygon_translation, 1.414, 0.1, false,true,true);

	scene.env->add(BulletSoftObject::Ptr(new BulletSoftObject(psb)));
	psb->scale(btVector3(1,1,1));
	psb->translate(btVector3(0,0,20));
	psb->setVolumeMass(300);

	///fix one vertex
	//psb->setMass(0,0);
	//psb->setMass(10,0);
	//psb->setMass(20,0);
	psb->m_cfg.piterations=1;
	//psb->generateClusters(128);
	psb->generateClusters(16);
	//psb->getCollisionShape()->setMargin(0.5);

	psb->getCollisionShape()->setMargin(0.01);
	psb->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS+	btSoftBody::fCollision::CL_RS
		+ btSoftBody::fCollision::CL_SELF
		;
	psb->m_materials[0]->m_kLST=0.8;

	//psb->generateBendingConstraints(2);

	psb->m_cfg.kDF=1.0;

	return psb;
}

//
// TetraPrism
//
void initTetraPrism(Scene& scene)
{
  vector<btVector3> corners_base(6);
  btVector3 polygon_translation;
  btSoftBody* psb;

  corners_base[0] = btVector3(0,0,0);
  corners_base[1] = btVector3(2,0,0);
  corners_base[2] = btVector3(3,1,0);
  corners_base[3] = btVector3(2,2,0);
  corners_base[4] = btVector3(0,2,0);
  corners_base[5] = btVector3(-1,1,0);
  polygon_translation = btVector3(0,4,-12);
  psb = makeTetraPrism(scene, corners_base, polygon_translation);
  psb->translate(btVector3(0,16,0));

  corners_base[0] = btVector3(0,0,0);
  corners_base[1] = btVector3(2,0,0);
  corners_base[2] = btVector3(3,1,0);
  corners_base[3] = btVector3(2,2,0);
  corners_base[4] = btVector3(0,2,0);
  corners_base[5] = btVector3(-1,1,0);
  polygon_translation = btVector3(0,4,12);
  psb = makeTetraPrism(scene, corners_base, polygon_translation);
  psb->translate(btVector3(0,8,0));

  corners_base[0] = btVector3(0,0,0);
  corners_base[1] = btVector3(2,0,0);
  corners_base[2] = btVector3(3,1,0);
  corners_base[3] = btVector3(2,2,0);
  corners_base[4] = btVector3(0,2,0);
  corners_base[5] = btVector3(-1,1,0);
  polygon_translation = btVector3(0,0,12);
  psb = makeTetraPrism(scene, corners_base, polygon_translation);
  psb->translate(btVector3(0,0,0));

  corners_base[0] = btVector3(0,0,0);
  corners_base[1] = btVector3(2,0,0);
  corners_base[2] = btVector3(3,1,0);
  corners_base[3] = btVector3(2,2,0);
  corners_base[4] = btVector3(0,2,0);
  corners_base[5] = btVector3(-1,1,0);
  polygon_translation = btVector3(4,0,12);
  psb = makeTetraPrism(scene, corners_base, polygon_translation);
  psb->translate(btVector3(0,-8,0));

  corners_base[0] = btVector3(0,0,0);
  corners_base[1] = btVector3(2,0,0);
  corners_base[2] = btVector3(3,1,0);
  corners_base[3] = btVector3(2,2,0);
  corners_base[4] = btVector3(0,2,0);
  corners_base[5] = btVector3(-1,1,0);
  polygon_translation = btVector3(4,0,-12);
  psb = makeTetraPrism(scene, corners_base, polygon_translation);
  psb->translate(btVector3(0,-16,0));
}


//
// 3DCloth
//
void	init3DCloth(Scene& scene)
{
	float half_extent = 14;
	vector<btVector3> corners_base;
	corners_base.push_back(btVector3(-half_extent,-half_extent,0));
	corners_base.push_back(btVector3(half_extent,-half_extent,0));
	corners_base.push_back(btVector3(half_extent,half_extent,0));
	corners_base.push_back(btVector3(-half_extent,half_extent,0));
	btVector3 polygon_translation = btVector3(0,0,4);

	//Create your psb
	btSoftBody* psb=CreatePrism(*scene.bullet->softBodyWorldInfo, corners_base, polygon_translation, 1.414, 0.4, false,true,true);

	scene.env->add(BulletSoftObject::Ptr(new BulletSoftObject(psb)));
	psb->scale(btVector3(1,1,1));
	psb->translate(btVector3(0,0,5));
	psb->setVolumeMass(300);

	///fix one vertex
	//psb->setMass(0,0);
	//psb->setMass(10,0);
	//psb->setMass(20,0);
	psb->m_cfg.piterations=1;
	//psb->generateClusters(128);
	psb->generateClusters(16);
	//psb->getCollisionShape()->setMargin(0.5);

	psb->getCollisionShape()->setMargin(0.01);
	psb->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS+	btSoftBody::fCollision::CL_RS
		+ btSoftBody::fCollision::CL_SELF
		;
	psb->m_materials[0]->m_kLST=0.8;

	//psb->generateBendingConstraints(2);

	psb->m_cfg.kDF=1.0;
}

//
// Cube Sponge
//
void	initSponge(Scene& scene)
{
	float half_extent = 1.4*METERS;
	vector<btVector3> corners_base;
	corners_base.push_back(btVector3(-half_extent,-half_extent,0));
	corners_base.push_back(btVector3(half_extent,-half_extent,0));
	corners_base.push_back(btVector3(half_extent,half_extent,0));
	corners_base.push_back(btVector3(-half_extent,half_extent,0));
	btVector3 polygon_translation = btVector3(0,0,0.3*METERS);

	//Create your psb
	btSoftBody* psb=CreatePrism(*scene.bullet->softBodyWorldInfo, corners_base, polygon_translation, 1.414, 0.8*METERS*METERS*METERS/1000.0, false,true,true);

	scene.env->add(BulletSoftObject::Ptr(new BulletSoftObject(psb)));
	//psb->scale(btVector3(METERS/10.0,METERS/10.0,METERS/10.0));
	psb->translate(btVector3(0,0,0.5*METERS));
	psb->setVolumeMass(300);

	///fix one vertex
	//psb->setMass(0,0);
	//psb->setMass(10,0);
	//psb->setMass(20,0);
	psb->m_cfg.piterations=1;
	//psb->generateClusters(128);
	psb->generateClusters(16);
	//psb->getCollisionShape()->setMargin(0.5);

	psb->getCollisionShape()->setMargin(0.001*METERS);
	psb->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS+	btSoftBody::fCollision::CL_RS
//		+ btSoftBody::fCollision::CL_SELF
		;
	psb->m_materials[0]->m_kLST=0.4;

	//psb->generateBendingConstraints(2);

	psb->m_cfg.kDF=1.0;
}

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 10.;
    BulletConfig::maxSubSteps = 0;

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);

    Scene scene;

    //initTetraFile(scene);
    //initTetraPrism(scene);
    //init3DCloth(scene);
    initSponge(scene);

    boost::shared_ptr<CapsuleRope> ropePtr;
	vector<btVector3> ctrlPts;
	int nLinks = 20;
	btScalar segment_len = 0.05, rope_radius = 0.05;
    for (int i=0; i< nLinks; i++)
    	ctrlPts.push_back(btVector3(5,5,1) + METERS*btVector3(.5+segment_len*i,0,5*rope_radius));

    ropePtr.reset(new CapsuleRope(ctrlPts,METERS*rope_radius));
    scene.env->add(ropePtr);
    ropePtr->setColor(0,1,0,1);


    scene.startViewer();
    scene.startLoop();
    return 0;
}

