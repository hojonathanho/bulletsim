#ifndef _collision_util_h
#define _collision_util_h
#include "simulation/simplescene.h"


struct Collision {
	const btCollisionObject* m_obj0;
	const btCollisionObject* m_obj1;
	btVector3 m_world0;
	btVector3 m_world1;
	btVector3 m_normal;
	btScalar m_distance;
	Collision(const btCollisionObject* obj0, const btCollisionObject* obj1, const btVector3& world0, const btVector3& world1, const btVector3& normal, btScalar distance) :
		m_obj0(obj0), m_obj1(obj1), m_world0(world0), m_world1(world1), m_normal(normal), m_distance(distance) {
	}
	Collision(const Collision& c) :
		m_obj0(c.m_obj0), m_obj1(c.m_obj1), m_world0(c.m_world0), m_world1(c.m_world1), m_normal(c.m_normal), m_distance(c.m_distance) {
	}
};

struct CollisionCollector: public btCollisionWorld::ContactResultCallback {
	std::vector<Collision> m_collisions;
	btScalar addSingleResult(btManifoldPoint& pt, const btCollisionObject *colObj0, int, int, const btCollisionObject *colObj1, int, int) {
		m_collisions.push_back(Collision(colObj0, colObj1, pt.m_positionWorldOnA, pt.m_positionWorldOnB, pt.m_normalWorldOnB, pt.m_distance1));
		return 0;
	}
};

inline void calcCollisionInfo(btRigidBody* body, btCollisionWorld* world, vector<btVector3>& points, vector<btVector3>& normals, vector<double>& dists) {
  CollisionCollector collisionCollector;
  world->contactTest(body, collisionCollector);
  int nColl = collisionCollector.m_collisions.size();
  points.resize(nColl);
  normals.resize(nColl);
  dists.resize(nColl);

  for (int iColl = 0; iColl < nColl; iColl++) {
    Collision &collision = collisionCollector.m_collisions[iColl];
    //cout << collision.m_world0 << " " << collision.m_world1 << endl;
    points[iColl] = (collision.m_obj0 == body) ? collision.m_world0 : collision.m_world1;
    normals[iColl] = (collision.m_obj0 == body) ? collision.m_normal : -collision.m_normal;
    dists[iColl] = collision.m_distance;
  }
}




#endif
