#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <iostream>
#include "utils/conversions.h"

struct LocalConfig : Config {
	static float kp;
	static float kd;
	static float ki;

  LocalConfig() : Config() {
    params.push_back(new Parameter<float>("kp", &kp, ""));
    params.push_back(new Parameter<float>("kd", &kd, ""));
    params.push_back(new Parameter<float>("ki", &ki, ""));
  }
};

float LocalConfig::kp = 1000;
float LocalConfig::kd = 10;
float LocalConfig::ki = 0.5;

BulletSoftObject::Ptr initCloth(btSoftBodyWorldInfo& world_info, btScalar sx, btScalar sy, int rx, int ry, btScalar z) {
	int fixed=0;//4+8;
	btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(world_info,btVector3(-sx,-sy,z),
		btVector3(+sx,-sy,z),
		btVector3(-sx,+sy,z),
		btVector3(+sx,+sy,z),rx,ry,fixed,true);

//	psb->getCollisionShape()->setMargin(0.5);
	btSoftBody::Material* pm=psb->appendMaterial();
	pm->m_kLST		=	0.001;
	psb->generateBendingConstraints(2,pm);
	psb->setTotalMass(0.1);

	psb->generateClusters(512);
	psb->getCollisionShape()->setMargin(0.002*METERS);

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

	BulletSoftObject::Ptr pso(new BulletSoftObject(psb));
	return pso;
}

btVector3 computeVelocity(const btVector3& x, const btVector3& target, const float& kp, const float& kd, btVector3 d_state, const float& ki=0, btVector3 i_state=btVector3(0,0,0)) {
	btVector3 x_err = target - x;

	btVector3 p_term = kp*x_err;

	btVector3 d_term = kd * (x_err - d_state);
	d_state = x_err;

//	I_state = I_state + a_err
//	if I_state > I_state_max:
//			I_state = I_state_max
//	elif I_state < I_state_min:
//			I_state = I_state_min
//	I_term = I_state * I_const

	return p_term + d_term;
}

void setVelocities(BulletSoftObject::Ptr cloth, btSoftBody::tNodeArray target_nodes, btScalar timeStep, const float& kp, const float& kd, vector<btVector3>& d_state) {
	btSoftBody::tNodeArray &nodes = cloth->softBody->m_nodes;
	for (int i=0; i<nodes.size(); i++) {
		btVector3 delta = target_nodes[i].m_x - nodes[i].m_x;
//		static const btScalar	maxdrag=0.01*METERS;
//		if(delta.length2()>(maxdrag*maxdrag))
//		{
//			delta = delta.normalized()*maxdrag;
//		}
		//nodes[i].m_v += kp*delta/timeStep;
		//nodes[i].m_v += computeVelocity(nodes[i].m_x, target_nodes[i].m_x, kp, kd, d_state[i]) / timeStep;
		//nodes[i].m_v = computeVelocity(nodes[i].m_x, target_nodes[i].m_x, kp, kd, nodes[i].m_v*timeStep) / timeStep;
		btVector3 x_err = target_nodes[i].m_x - nodes[i].m_x;
		//nodes[i].m_v += (kp * x_err + kd * (x_err - nodes[i].m_v*timeStep))/timeStep;
		cloth->softBody->addForce((kp*x_err-kd*nodes[i].m_v)/nodes[i].m_im, i);
		cout << nodes[i].m_im << endl;
	}
}

void shift(BulletSoftObject::Ptr pso, btVector3 increment) {
	btSoftBody::tNodeArray &nodes = pso->softBody->m_nodes;
	for (int i=0; i<nodes.size(); i++) {
		nodes[i].m_x += increment;
	}
}

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 100.;
    BulletConfig::maxSubSteps = 0;

    Parser parser;
		parser.addGroup(GeneralConfig());
		parser.addGroup(BulletConfig());
		parser.addGroup(SceneConfig());
		parser.addGroup(LocalConfig());
		parser.read(argc, argv);

    Scene scene;

  	const btScalar	s=0.1*METERS;
		const btScalar	h=0.25*METERS;
		const int		r=30;

		btTransform startTransform; btCollisionShape* capsuleShape;	btCollisionShape* planeShape;	btRigidBody* body;

		BoxObject::Ptr plane(new BoxObject(0, btVector3(s*2,s*2,0.01*METERS), btTransform(btQuaternion(btVector3(1,0,0), -10.0 * M_PI/180.0), btVector3(0,0,h-0.15*METERS))));
		plane->collisionShape->setMargin(0.001*METERS);
		plane->rigidBody->setFriction(1.0);
		scene.env->add(plane);

		CapsuleObject::Ptr capsule(new CapsuleObject(0, 0.01*METERS,s*2, btTransform(btQuaternion(btVector3(0,0,1), 0.0), btVector3(0,-s*0.5,h-0.04*METERS))));
		capsule->collisionShape->setMargin(0.005*METERS);
		capsule->rigidBody->setFriction(0.0);
		scene.env->add(capsule);

		BulletSoftObject::Ptr cloth = initCloth(scene.env->bullet->softBodyWorldInfo, s, s, r, r, h);
		cloth->setColor(0,1,0,1);
		scene.env->add(cloth);

		BulletSoftObject::Ptr cloth_static = initCloth(scene.env->bullet->softBodyWorldInfo, s, s, r, r, h);
		//cloth_static->softBody->setTotalMass(0);
		for (int i=0; i<cloth_static->softBody->m_nodes.size(); i++) {
			cloth_static->softBody->setMass(i,0);
		}
		cloth_static->softBody->m_cfg.collisions = 0;
		cloth_static->setColor(1,0,0,1);
		scene.env->add(cloth_static);

		scene.addVoidKeyCallback('i',boost::bind(shift, cloth_static, btVector3(0,0,0.01*METERS)));
		scene.addVoidKeyCallback('k',boost::bind(shift, cloth_static, btVector3(0,0,-0.01*METERS)));
		scene.addVoidKeyCallback('j',boost::bind(shift, cloth_static, btVector3(0,-0.01*METERS,0)));
		scene.addVoidKeyCallback('l',boost::bind(shift, cloth_static, btVector3(0,0.01*METERS,0)));
	  scene.addVoidKeyCallback('q',boost::bind(exit, 0));

		btSoftBody::tNodeArray& target_nodes = cloth_static->softBody->m_nodes;
		vector<btVector3> d_states(target_nodes.size(), btVector3(0,0,0));
//	  I_state = 0
//	  I_state_max = 500
//	  I_state_min = -500

		scene.startViewer();

    boost::posix_time::ptime sim_time = boost::posix_time::microsec_clock::local_time();
    while (true) {
    	setVelocities(cloth, target_nodes, 0.03, LocalConfig::kp, LocalConfig::kd, d_states);
    	scene.step(0.03);
    	cout << (boost::posix_time::microsec_clock::local_time() - sim_time).total_milliseconds() << endl;
    	sim_time = boost::posix_time::microsec_clock::local_time();
    }

    return 0;
}
