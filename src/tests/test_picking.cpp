#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

#include <stdio.h>
#include <iostream>
#include "utils/conversions.h"

void initCloth(Scene &scene, btScalar s, btScalar z) {
	const int		r=30;
	int fixed=0;//4+8;
	btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(*scene.env->bullet->softBodyWorldInfo,btVector3(-s,-s,z),
		btVector3(+s,-s,z),
		btVector3(-s,+s,z),
		btVector3(+s,+s,z),r,r,fixed,true);

//	psb->getCollisionShape()->setMargin(0.5);
	btSoftBody::Material* pm=psb->appendMaterial();
	pm->m_kLST		=	0.001;
	psb->generateBendingConstraints(2,pm);
	psb->setTotalMass(150);

	psb->generateClusters(100);
	psb->getCollisionShape()->setMargin(0.2);

	psb->m_cfg.collisions	=	0;
	psb->m_cfg.collisions += btSoftBody::fCollision::SDF_RS; ///SDF based rigid vs soft
	//psb->m_cfg.collisions += btSoftBody::fCollision::CL_RS; ///Cluster vs convex rigid vs soft
	//psb->m_cfg.collisions += btSoftBody::fCollision::VF_SS;	///Vertex vs face soft vs soft handling
	psb->m_cfg.collisions += btSoftBody::fCollision::CL_SS; ///Cluster vs cluster soft vs soft handling
	psb->m_cfg.collisions	+= btSoftBody::fCollision::CL_SELF; ///Cluster soft body self collision

	psb->m_cfg.kDF = 1;

	psb->m_cfg.piterations = 50;
	psb->m_cfg.citerations = 50;
	psb->m_cfg.diterations = 50;
//	psb->m_cfg.viterations = 10;

	psb->randomizeConstraints();

	BulletSoftObject::Ptr cloth(new BulletSoftObject(psb));
	cloth->setColor(0,1,0,1);
	scene.env->add(cloth);
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

    initCloth(scene, 0.5*METERS, 1*METERS);

    BoxObject::Ptr box(new BoxObject(100, btVector3(0.25*METERS, 0.25*METERS, 0.10*METERS), btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0.2*METERS))));
    box->collisionShape->setMargin(0.1);
    box->rigidBody->setFriction(0.1);
    box->setColor(1,0,0,1);
    scene.env->add(box);

    scene.startViewer();
    scene.startLoop();
    return 0;
}
