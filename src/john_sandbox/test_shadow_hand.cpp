
#include "simulation/simplescene.h"
#include <Eigen/Dense>
#include <fstream>
#include <cstdio>

using namespace std;
using namespace Eigen;

typedef Matrix<uint8_t,Dynamic,Dynamic> MatrixXb;



struct LocalConfig : Config {
  static bool prepare;
  LocalConfig() : Config() {
    params.push_back(new Parameter<bool>("prepare",&prepare,"prepare overlap matrix"));
  }
};
bool LocalConfig::prepare = false;



typedef pair<const btRigidBody*, const btRigidBody*> RBPair;
struct CollisionCollectorCallback : public btCollisionWorld::ContactResultCallback {
	set<RBPair> pairs;
	btScalar addSingleResult(btManifoldPoint &,
							 const btCollisionObject *colObj0, int, int,
							 const btCollisionObject *colObj1, int, int) {
		pairs.insert(RBPair(dynamic_cast<const btRigidBody*>(colObj0), dynamic_cast<const btRigidBody*>(colObj1)));
		return 0;
	}
};




struct MatrixFilterCallback : public btOverlapFilterCallback {
  MatrixXb m_disabled;
  MatrixFilterCallback(const MatrixXb& disabled) : m_disabled(disabled) {} 
	bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
	{
    return !m_disabled(proxy0->m_collisionFilterGroup, proxy1->m_collisionFilterGroup);
	}
};

MatrixFilterCallback* filterCallback;


void MyNearCallback(btBroadphasePair& collisionPair,
  btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo) {

    // Do your collision logic here
    // Only dispatch the Bullet collision information if you want the physics to continue
    if (filterCallback->needBroadphaseCollision(collisionPair.m_pProxy0, collisionPair.m_pProxy1)) dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
}




void matrixToFile(const string& fname, const MatrixXb& mat) {
  ofstream outfile(fname.c_str());
  assert (!outfile.fail());

  for (int i=0; i < mat.rows(); ++i) {
    for (int j=0; j < mat.cols(); ++j)
      outfile << (int)mat(i,j) << " ";
    outfile << endl;
  }    
  
  outfile.close();
}


void matrixFromFile(const string& fname, MatrixXb& mat) {
  ifstream infile(fname.c_str());
  for (int i=0; i < mat.rows(); ++i)
    for (int j=0; j < mat.cols(); ++j) {
      int b;
      infile >> b;
      mat(i,j) = b;      
    }
      
  assert (!infile.fail());
  infile.close();
}


int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);

  Scene scene;
  
  
  
//  Load(scene.env, scene.rave, "/home/joschu/bulletsim/src/john_sandbox/shadow_hand.env.xml",true);
  Load(scene.env, scene.rave, "robots/pr2-beta-static.zae");
  
  btCollisionObjectArray& rbs = scene.env->bullet->dynamicsWorld->getCollisionObjectArray();
  for (int i=0; i < rbs.size(); ++i) {
    rbs[i]->getBroadphaseHandle()->m_collisionFilterGroup = i;
  }
  MatrixXb overlap = MatrixXb::Zero(rbs.size(), rbs.size());



  if (LocalConfig::prepare) {
  	CollisionCollectorCallback collector;
  	for (int i=0; i < rbs.size(); ++i) {
  			scene.env->bullet->dynamicsWorld->contactTest(rbs[i],collector);
  	}

  	BOOST_FOREACH(const RBPair& pair, collector.pairs) {
      overlap(pair.first->getBroadphaseProxy()->m_collisionFilterGroup, pair.second->getBroadphaseProxy()->m_collisionFilterGroup) = true;
  	}
  	
    matrixToFile("/tmp/overlap.txt", overlap);
  }
  
  else {
    matrixFromFile("/tmp/overlap.txt", overlap);
    filterCallback = new MatrixFilterCallback(overlap);  
    
//    scene.env->bullet->dynamicsWorld->getPairCache()->setOverlapFilterCallback(filterCallback);
    scene.env->bullet->dispatcher->setNearCallback(&MyNearCallback);
    scene.startViewer();
    scene.startLoop();
    
  }
	
}

