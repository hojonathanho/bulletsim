#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>

#include <stdio.h>
#include <iostream>
#include "utils/conversions.h"

boost::shared_mutex mutex;
boost::shared_mutex bullet_mutex;

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

boost::posix_time::ptime sim_time = boost::posix_time::microsec_clock::local_time();
void simulation(Scene* scene)
{
	int i=0;
  while (1) {
  	//boost::shared_lock< boost::shared_mutex > lock(bullet_mutex);
  	boost::unique_lock< boost::shared_mutex > lock(bullet_mutex);
  	boost::posix_time::time_duration sim_diff = boost::posix_time::microsec_clock::local_time() - sim_time;
  	sim_time = boost::posix_time::microsec_clock::local_time();
  	cout << sim_diff.total_milliseconds()<< " simulation iter " << i++;
		cout.flush();
		boost::posix_time::ptime step_time = boost::posix_time::microsec_clock::local_time();
		scene->step(0.03);
		boost::posix_time::time_duration step_diff = boost::posix_time::microsec_clock::local_time() - step_time;
		//usleep(20000);
		cout << " simulation just finished" << endl;
		lock.unlock();

		//waiting for rate
		if (step_diff.total_milliseconds() < 40)
		usleep(40000 - ((double)step_diff.total_milliseconds())*1000);
  }
}

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 100.;
    BulletConfig::maxSubSteps = 0;

    Parser parser;
		parser.addGroup(GeneralConfig());
		parser.addGroup(BulletConfig());
		parser.addGroup(SceneConfig());
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

//    BoxObject::Ptr box(new BoxObject(1, btVector3(0.25*METERS, 0.25*METERS, 0.10*METERS), btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0.2*METERS))));
//    box->collisionShape->setMargin(0.1);
//    box->rigidBody->setFriction(0.1);
//    box->setColor(1,0,0,1);
//    scene.env->add(box);

  	scene.startViewer();

  	boost::shared_ptr<boost::thread> deviceThread_ = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&simulation, &scene)));

  	boost::posix_time::ptime computing_time = boost::posix_time::microsec_clock::local_time();
  	boost::posix_time::ptime applying_time = boost::posix_time::microsec_clock::local_time();
    int i=0;
    while (1) {
			//reading and computing 500000
			boost::shared_lock< boost::shared_mutex > shared_lock(bullet_mutex);
    	//boost::upgrade_lock<boost::shared_mutex> shared_lock(bullet_mutex);
    	boost::posix_time::time_duration computing_diff = boost::posix_time::microsec_clock::local_time() - computing_time;
			computing_time = boost::posix_time::microsec_clock::local_time();
			cout << computing_diff.total_milliseconds() << " computing iter " << i;
			cout.flush();
			usleep(10000);
			cout << " computing just finished" << endl;
			shared_lock.unlock();

			//corr computing
			usleep(50000);

			//applying 10000
			boost::unique_lock< boost::shared_mutex > lock(bullet_mutex);
			//boost::upgrade_to_unique_lock<boost::shared_mutex> lock(shared_lock);
			boost::posix_time::time_duration applying_diff = boost::posix_time::microsec_clock::local_time() - applying_time;
			applying_time = boost::posix_time::microsec_clock::local_time();
			cout << applying_diff.total_milliseconds() << " applying iter " << i++;
			cout.flush();
			usleep(10000);
			cout << " applying just finished" << endl;
			lock.unlock();

			//sigma computing
			usleep(10000);
     }
    //scene.startLoop();
    return 0;
}
