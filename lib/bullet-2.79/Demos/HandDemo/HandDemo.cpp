/*
Bullet Continuous Collision Detection and Physics Library
Finger Demo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#define CONSTRAINT_DEBUG_SIZE 0.2f


#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "HandDemo.h"

#include <iostream>
#include <vector>
using namespace std;

// Enrico: Shouldn't these three variables be real constants and not defines?

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

#ifndef M_PI_8
#define M_PI_8     0.39269908169872414
#endif

#ifndef METERS
#define METERS     10
#endif

class GenericBodyPart
{
public:
	btDynamicsWorld* m_ownerWorld;
	vector<btCollisionShape*> m_shapes;
	vector<btRigidBody*> m_bodies;
	vector<btTypedConstraint*> m_joints;

	GenericBodyPart (btDynamicsWorld* ownerWorld) : m_ownerWorld (ownerWorld) {}

	virtual	~GenericBodyPart ()
	{
		int i;

		// Remove all constraints
		for ( i = 0; i < m_joints.size(); ++i)
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = 0;
		}

		// Remove all bodies
		for ( i = 0; i < m_bodies.size(); ++i)
		{
			m_ownerWorld->removeRigidBody(m_bodies[i]);
			delete m_bodies[i]->getMotionState();
			delete m_bodies[i]; m_bodies[i] = 0;
		}

		// Remove all shapes
		for ( i = 0; i < m_shapes.size(); ++i)
			delete m_shapes[i]; m_shapes[i] = 0;
	}

	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_ownerWorld->addRigidBody(body);

		return body;
	}
};

class Finger : public GenericBodyPart
{
public:
	enum
	{
		PART_PROXIMAL = 0,
		PART_MIDDLE,
		PART_DISTAL,

		PART_COUNT
	};

private:
	enum
	{
		JOINT_PROXIMAL = 0,
		JOINT_DISTAL,

		JOINT_COUNT
	};

public:
	Finger (btDynamicsWorld* ownerWorld, const btTransform& offset, vector<float> radii, vector<float> heights)
		: GenericBodyPart(ownerWorld)
	{
		assert(radii.size() == PART_COUNT);
		assert(heights.size() == PART_COUNT);

		m_shapes.resize(PART_COUNT);
		m_bodies.resize(PART_COUNT);
		m_joints.resize(JOINT_COUNT);

		// Setup the geometry
		for (int i = 0; i < PART_COUNT; ++i)
			m_shapes[i] = new btCapsuleShape(radii[i], heights[i]);

		// Setup all the rigid bodies
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_PROXIMAL]/2.0), btScalar(0.)));
		m_bodies[PART_PROXIMAL] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[PART_PROXIMAL]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_PROXIMAL] - heights[PART_MIDDLE]/2.0), btScalar(0.)));
		m_bodies[PART_MIDDLE] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[PART_MIDDLE]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_PROXIMAL] - heights[PART_MIDDLE] - heights[PART_DISTAL]/2.0), btScalar(0.)));
		m_bodies[PART_DISTAL] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[PART_DISTAL]);

		// Setup some damping on the m_bodies
		for (int i = 0; i < PART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(0.05, 0.85);
			m_bodies[i]->setDeactivationTime(0.8);
			m_bodies[i]->setSleepingThresholds(1.6, 2.5);
		}

		// Now setup the constraints
		btHingeConstraint* hingeC;

		btTransform localA, localB;

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_PROXIMAL]/2.0), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(heights[PART_MIDDLE]/2.0), btScalar(0.)));
		//localA = btTransform(m_bodies[PART_PROXIMAL]->getCenterOfMassTransform().getBasis(), btVector3(0,0,0))
		//		* btTransform(btQuaternion::getIdentity(), btVector3(0,-heights[PART_PROXIMAL]/2.0,0));
		hingeC =  new btHingeConstraint(*m_bodies[PART_PROXIMAL], *m_bodies[PART_MIDDLE], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_PROXIMAL] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_PROXIMAL], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_MIDDLE]/2.0), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(heights[PART_DISTAL]/2.0), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[PART_MIDDLE], *m_bodies[PART_DISTAL], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_PROXIMAL] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_PROXIMAL], true);
	}
};


class Thumb : public GenericBodyPart
{
public:
	enum
	{
		PART_PROXIMAL = 0,
		PART_DISTAL,

		PART_COUNT
	};

private:
	enum
	{
		JOINT_PROXIMAL = 0,

		JOINT_COUNT
	};

public:
	Thumb (btDynamicsWorld* ownerWorld, const btTransform& offset, vector<float> radii, vector<float> heights)
		: GenericBodyPart(ownerWorld)
	{
		assert(radii.size() == PART_COUNT);
		assert(heights.size() == PART_COUNT);

		m_shapes.resize(PART_COUNT);
		m_bodies.resize(PART_COUNT);
		m_joints.resize(JOINT_COUNT);

		// Setup the geometry
		for (int i = 0; i < PART_COUNT; ++i)
			m_shapes[i] = new btCapsuleShape(radii[i], heights[i]);

		// Setup all the rigid bodies
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_PROXIMAL]/2.0), btScalar(0.)));
		m_bodies[PART_PROXIMAL] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[PART_PROXIMAL]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_PROXIMAL] - heights[PART_DISTAL]/2.0), btScalar(0.)));
		m_bodies[PART_DISTAL] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[PART_DISTAL]);

		// Setup some damping on the m_bodies
		for (int i = 0; i < PART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(0.05, 0.85);
			m_bodies[i]->setDeactivationTime(0.8);
			m_bodies[i]->setSleepingThresholds(1.6, 2.5);
		}

		// Now setup the constraints
		btHingeConstraint* hingeC;

		btTransform localA, localB;

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_PROXIMAL]/2.0), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(heights[PART_DISTAL]/2.0), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[PART_PROXIMAL], *m_bodies[PART_DISTAL], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_PROXIMAL] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_PROXIMAL], true);
	}
};

class Palm : public GenericBodyPart
{
public:
	enum
	{
		PART_THUMB = 0,
		PART_INDEX,
		PART_MIDDLE,
		PART_RING,
		PART_LITTLE,

		PART_COUNT
	};

private:
	enum
	{
		JOINT_THUMB_INDEX = 0,
		JOINT_INDEX_MIDDLE,
		JOINT_MIDDLE_RING,
		JOINT_RING_LITTLE,

		JOINT_COUNT
	};

public:
	Palm (btDynamicsWorld* ownerWorld, const btTransform& offset, vector<float> radii, vector<float> heights)
	: GenericBodyPart(ownerWorld)
	{
		assert(radii.size() == PART_COUNT);
		assert(heights.size() == PART_COUNT);

		m_shapes.resize(PART_COUNT);
		m_bodies.resize(PART_COUNT);
		m_joints.resize(JOINT_COUNT);

		// Setup the geometry
		for (int i = 0; i < PART_COUNT; ++i)
			m_shapes[i] = new btCapsuleShape(radii[i], heights[i]);


		// Setup all the rigid bodies
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-radii[PART_THUMB] - 2.0*radii[PART_INDEX] - radii[PART_MIDDLE]), -heights[PART_THUMB]/2.0, btScalar(0.)));
		m_bodies[PART_THUMB] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[PART_THUMB]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-radii[PART_INDEX] - radii[PART_MIDDLE]), -heights[PART_INDEX]/2.0, btScalar(0.)));
		m_bodies[PART_INDEX] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[PART_INDEX]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), -heights[PART_MIDDLE]/2.0, btScalar(0.)));
		m_bodies[PART_MIDDLE] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[PART_MIDDLE]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(radii[PART_INDEX] + radii[PART_RING]), -heights[PART_RING]/2.0, btScalar(0.)));
		m_bodies[PART_RING] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[PART_RING]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(radii[PART_INDEX] + 2.0*radii[PART_RING] + radii[PART_LITTLE]), -heights[PART_LITTLE]/2.0, btScalar(0.)));
		m_bodies[PART_LITTLE] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[PART_LITTLE]);


		// Setup some damping on the m_bodies
		for (int i = 0; i < PART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(0.05, 0.85);
			m_bodies[i]->setDeactivationTime(0.8);
			m_bodies[i]->setSleepingThresholds(1.6, 2.5);
		}

		// Now setup the constraints
		btHingeConstraint* hingeC;

		btTransform localA, localB;

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localA.setOrigin(btVector3(btScalar(radii[PART_THUMB]), heights[PART_THUMB]/2.0, btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localB.setOrigin(btVector3(btScalar(-radii[PART_INDEX]), heights[PART_INDEX]/2.0, btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[PART_THUMB], *m_bodies[PART_INDEX], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_8));
		m_joints[JOINT_THUMB_INDEX] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_THUMB_INDEX], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localA.setOrigin(btVector3(btScalar(radii[PART_INDEX]), heights[PART_INDEX]/2.0, btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localB.setOrigin(btVector3(btScalar(-radii[PART_MIDDLE]), heights[PART_MIDDLE]/2.0, btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[PART_INDEX], *m_bodies[PART_MIDDLE], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_8));
		m_joints[JOINT_INDEX_MIDDLE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_INDEX_MIDDLE], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localA.setOrigin(btVector3(btScalar(radii[PART_MIDDLE]), heights[PART_MIDDLE]/2.0, btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localB.setOrigin(btVector3(btScalar(-radii[PART_RING]), heights[PART_RING]/2.0, btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[PART_MIDDLE], *m_bodies[PART_RING], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_8));
		m_joints[JOINT_MIDDLE_RING] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_MIDDLE_RING], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localA.setOrigin(btVector3(btScalar(radii[PART_RING]), heights[PART_RING]/2.0, btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localB.setOrigin(btVector3(btScalar(-radii[PART_LITTLE]), heights[PART_LITTLE]/2.0, btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[PART_RING], *m_bodies[PART_LITTLE], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_8));
		m_joints[JOINT_RING_LITTLE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_RING_LITTLE], true);
	}
};

class Hand
{
public:
	enum
	{
		PART_PALM = 0,
		PART_THUMB,
		PART_INDEX,
		PART_MIDDLE,
		PART_RING,
		PART_LITTLE,

		PART_COUNT
	};

private:
	enum
	{
		JOINT_THUMB_KNUCKLES = 0,
		JOINT_INDEX_KNUCKLES,
		JOINT_MIDDLE_KNUCKLES,
		JOINT_RING_KNUCKLES,
		JOINT_LITTLE_KNUCKLES,

		JOINT_COUNT
	};

	btDynamicsWorld* m_ownerWorld;
	vector<btTypedConstraint*> m_joints;
	vector<GenericBodyPart*> m_parts;

public:
	Hand (btDynamicsWorld* ownerWorld, const btTransform& offset, vector<vector<float> > radii, vector<vector<float> > heights)
	: m_ownerWorld(ownerWorld)
	{
		assert(radii.size() == PART_COUNT);
		assert(heights.size() == PART_COUNT);

		m_parts.resize(PART_COUNT);
		m_joints.resize(JOINT_COUNT);

		// Setup all the hand parts
		btTransform transform;
		transform.setIdentity();
		m_parts[PART_PALM]   = new Palm(ownerWorld, offset*transform, radii[PART_PALM], heights[PART_PALM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-radii[Hand::PART_PALM][Palm::PART_THUMB] - 2.0*radii[Hand::PART_PALM][Palm::PART_INDEX] - radii[Hand::PART_PALM][Palm::PART_MIDDLE]),
				 btScalar(-heights[Hand::PART_PALM][Palm::PART_THUMB]),
				 btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_8);
		m_parts[PART_THUMB]  = new Thumb(ownerWorld, offset*transform, radii[PART_THUMB], heights[PART_THUMB]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-radii[Hand::PART_PALM][Palm::PART_INDEX] - radii[Hand::PART_PALM][Palm::PART_MIDDLE]),
				 btScalar(-heights[Hand::PART_PALM][Palm::PART_INDEX]),
				 btScalar(0.)));
		m_parts[PART_INDEX]  = new Finger(ownerWorld, offset*transform, radii[PART_INDEX], heights[PART_INDEX]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.),
				 btScalar(-heights[Hand::PART_PALM][Palm::PART_MIDDLE]),
				 btScalar(0.)));
		m_parts[PART_MIDDLE] = new Finger(ownerWorld, offset*transform, radii[PART_MIDDLE], heights[PART_MIDDLE]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(radii[Hand::PART_PALM][Palm::PART_MIDDLE] + radii[Hand::PART_PALM][Palm::PART_RING]),
				 btScalar(-heights[Hand::PART_PALM][Palm::PART_RING]),
				 btScalar(0.)));
		m_parts[PART_RING]   = new Finger(ownerWorld, offset*transform, radii[PART_RING], heights[PART_RING]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(radii[Hand::PART_PALM][Palm::PART_MIDDLE] + 2.0*radii[Hand::PART_PALM][Palm::PART_RING] + radii[Hand::PART_PALM][Palm::PART_LITTLE]),
				 btScalar(-heights[Hand::PART_PALM][Palm::PART_LITTLE]),
				 btScalar(0.)));
		m_parts[PART_LITTLE] = new Finger(ownerWorld, offset*transform, radii[PART_LITTLE], heights[PART_LITTLE]);

		// Now setup the constraints
		btHingeConstraint* hingeC;

		btTransform localA, localB;

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,-M_PI_8); localA.setOrigin(btVector3(btScalar(0.), -heights[Hand::PART_PALM][Palm::PART_THUMB]/2.0, btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), heights[Hand::PART_THUMB][Thumb::PART_PROXIMAL]/2.0, btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_parts[Hand::PART_PALM]->m_bodies[Palm::PART_THUMB], *m_parts[Hand::PART_THUMB]->m_bodies[Thumb::PART_PROXIMAL], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_THUMB_KNUCKLES] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_THUMB_KNUCKLES], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), -heights[Hand::PART_PALM][Palm::PART_INDEX]/2.0, btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), heights[Hand::PART_INDEX][Finger::PART_PROXIMAL]/2.0, btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_parts[Hand::PART_PALM]->m_bodies[Palm::PART_INDEX], *m_parts[Hand::PART_INDEX]->m_bodies[Finger::PART_PROXIMAL], localA, localB);
		hingeC->setLimit(btScalar(-M_PI_8), btScalar(M_PI_2));
		m_joints[JOINT_INDEX_KNUCKLES] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_INDEX_KNUCKLES], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), -heights[Hand::PART_PALM][Palm::PART_MIDDLE]/2.0, btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), heights[Hand::PART_MIDDLE][Finger::PART_PROXIMAL]/2.0, btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_parts[Hand::PART_PALM]->m_bodies[Palm::PART_MIDDLE], *m_parts[Hand::PART_MIDDLE]->m_bodies[Finger::PART_PROXIMAL], localA, localB);
		hingeC->setLimit(btScalar(-M_PI_8), btScalar(M_PI_2));
		m_joints[JOINT_MIDDLE_KNUCKLES] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_MIDDLE_KNUCKLES], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), -heights[Hand::PART_PALM][Palm::PART_RING]/2.0, btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), heights[Hand::PART_RING][Finger::PART_PROXIMAL]/2.0, btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_parts[Hand::PART_PALM]->m_bodies[Palm::PART_RING], *m_parts[Hand::PART_RING]->m_bodies[Finger::PART_PROXIMAL], localA, localB);
		hingeC->setLimit(btScalar(-M_PI_8), btScalar(M_PI_2));
		m_joints[JOINT_RING_KNUCKLES] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_RING_KNUCKLES], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), -heights[Hand::PART_PALM][Palm::PART_LITTLE]/2.0, btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), heights[Hand::PART_LITTLE][Finger::PART_PROXIMAL]/2.0, btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_parts[Hand::PART_PALM]->m_bodies[Palm::PART_LITTLE], *m_parts[Hand::PART_LITTLE]->m_bodies[Finger::PART_PROXIMAL], localA, localB);
		hingeC->setLimit(btScalar(-M_PI_8), btScalar(M_PI_2));
		m_joints[JOINT_LITTLE_KNUCKLES] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_LITTLE_KNUCKLES], true);
	}

	virtual	~Hand ()
	{
		int i;

		// Remove all constraints
		for ( i = 0; i < m_joints.size(); ++i)
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = 0;
		}

		// Remove all parts
		for ( i = 0; i < m_parts.size(); ++i)
			delete m_parts[i];
	}

};

void HandDemo::initPhysics()
{
	// Setup the basic world

	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(5.));

	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
	//m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;



	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-10,0));

#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
		btCollisionObject* fixedGround = new btCollisionObject();
		fixedGround->setCollisionShape(groundShape);
		fixedGround->setWorldTransform(groundTransform);
		m_dynamicsWorld->addCollisionObject(fixedGround);
#else
		localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
#endif //CREATE_GROUND_COLLISION_OBJECT

	}

	// Spawn one finger
	btTransform startOffset; startOffset.setIdentity();
	startOffset.setOrigin(btVector3(0.15*METERS,0.30*METERS,0));
	spawnHand(startOffset);
	startOffset.setOrigin(btVector3(-0.15*METERS,0.30*METERS,0));
	spawnHand(startOffset);

	clientResetScene();		
}

void HandDemo::spawnHand(const btTransform& startOffset)
{
	vector<vector<float> > radii(Hand::PART_COUNT);

	radii[Hand::PART_PALM] = vector<float>(Palm::PART_COUNT);
	radii[Hand::PART_PALM][Palm::PART_THUMB]  = 0.024 * 0.5 * METERS;
	radii[Hand::PART_PALM][Palm::PART_INDEX]  = 0.022 * 0.5 * METERS;
	radii[Hand::PART_PALM][Palm::PART_MIDDLE] = 0.022 * 0.5 * METERS;
	radii[Hand::PART_PALM][Palm::PART_RING]   = 0.02 * 0.5 * METERS;
	radii[Hand::PART_PALM][Palm::PART_LITTLE] = 0.018 * 0.5 * METERS;

	radii[Hand::PART_THUMB] = vector<float>(Thumb::PART_COUNT);
	radii[Hand::PART_THUMB][Thumb::PART_PROXIMAL] = 0.022 * 0.5 * METERS;
	radii[Hand::PART_THUMB][Thumb::PART_DISTAL]   = 0.023 * 0.5 * METERS;

	radii[Hand::PART_INDEX] = vector<float>(Finger::PART_COUNT);
	radii[Hand::PART_INDEX][Finger::PART_PROXIMAL] = 0.02 * 0.5 * METERS;
	radii[Hand::PART_INDEX][Finger::PART_MIDDLE]   = 0.016 * 0.5 * METERS;
	radii[Hand::PART_INDEX][Finger::PART_DISTAL]   = 0.013 * 0.5 * METERS;

	radii[Hand::PART_MIDDLE] = vector<float>(Finger::PART_COUNT);
	radii[Hand::PART_MIDDLE][Finger::PART_PROXIMAL] = 0.02 * 0.5 * METERS;
	radii[Hand::PART_MIDDLE][Finger::PART_MIDDLE]   = 0.013 * 0.5 * METERS;
	radii[Hand::PART_MIDDLE][Finger::PART_DISTAL]   = 0.012 * 0.5 * METERS;

	radii[Hand::PART_RING] = vector<float>(Finger::PART_COUNT);
	radii[Hand::PART_RING][Finger::PART_PROXIMAL] = 0.018 * 0.5 * METERS;
	radii[Hand::PART_RING][Finger::PART_MIDDLE]   = 0.015 * 0.5 * METERS;
	radii[Hand::PART_RING][Finger::PART_DISTAL]   = 0.013 * 0.5 * METERS;

	radii[Hand::PART_LITTLE] = vector<float>(Finger::PART_COUNT);
	radii[Hand::PART_LITTLE][Finger::PART_PROXIMAL] = 0.016 * 0.5 * METERS;
	radii[Hand::PART_LITTLE][Finger::PART_MIDDLE]   = 0.014 * 0.5 * METERS;
	radii[Hand::PART_LITTLE][Finger::PART_DISTAL]   = 0.013 * 0.5 * METERS;


	vector<vector<float> > heights(Hand::PART_COUNT);

	heights[Hand::PART_PALM] = vector<float>(Palm::PART_COUNT);
	heights[Hand::PART_PALM][Palm::PART_THUMB]  = 0.09 * METERS;
	heights[Hand::PART_PALM][Palm::PART_INDEX]  = 0.098 * METERS;
	heights[Hand::PART_PALM][Palm::PART_MIDDLE] = 0.098 * METERS;
	heights[Hand::PART_PALM][Palm::PART_RING]   = 0.088 * METERS;
	heights[Hand::PART_PALM][Palm::PART_LITTLE] = 0.081 * METERS;

	heights[Hand::PART_THUMB] = vector<float>(Thumb::PART_COUNT);
	heights[Hand::PART_THUMB][Thumb::PART_PROXIMAL] = 0.047 * METERS;
	heights[Hand::PART_THUMB][Thumb::PART_DISTAL]   = 0.039 * METERS;

	heights[Hand::PART_INDEX] = vector<float>(Finger::PART_COUNT);
	heights[Hand::PART_INDEX][Finger::PART_PROXIMAL] = 0.06 * METERS;
	heights[Hand::PART_INDEX][Finger::PART_MIDDLE]   = 0.036 * METERS;
	heights[Hand::PART_INDEX][Finger::PART_DISTAL]   = 0.029 * METERS;

	heights[Hand::PART_MIDDLE] = vector<float>(Finger::PART_COUNT);
	heights[Hand::PART_MIDDLE][Finger::PART_PROXIMAL] = 0.063 * METERS;
	heights[Hand::PART_MIDDLE][Finger::PART_MIDDLE]   = 0.039 * METERS;
	heights[Hand::PART_MIDDLE][Finger::PART_DISTAL]   = 0.029 * METERS;

	heights[Hand::PART_RING] = vector<float>(Finger::PART_COUNT);
	heights[Hand::PART_RING][Finger::PART_PROXIMAL] = 0.056 * METERS;
	heights[Hand::PART_RING][Finger::PART_MIDDLE]   = 0.034 * METERS;
	heights[Hand::PART_RING][Finger::PART_DISTAL]   = 0.022 * METERS;

	heights[Hand::PART_LITTLE] = vector<float>(Finger::PART_COUNT);
	heights[Hand::PART_LITTLE][Finger::PART_PROXIMAL] = 0.049 * METERS;
	heights[Hand::PART_LITTLE][Finger::PART_MIDDLE]   = 0.027 * METERS;
	heights[Hand::PART_LITTLE][Finger::PART_DISTAL]   = 0.025 * METERS;


	Hand* hand = new Hand (m_dynamicsWorld, startOffset, radii, heights);
	m_hands.push_back(hand);
}	

void HandDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't fingerle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();


	}

	renderme(); 

	glFlush();

	glutSwapBuffers();
}

void HandDemo::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void HandDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'e':
		{
		btTransform startOffset; startOffset.setIdentity();
		startOffset.setOrigin(btVector3(0,0.30*METERS,0));
		spawnHand(startOffset);
		break;
		}
	default:
		DemoApplication::keyboardCallback(key, x, y);
	}

	
}



void	HandDemo::exitPhysics()
{

	int i;

	for (i=0;i<m_hands.size();i++)
	{
		Hand* doll = m_hands[i];
		delete doll;
	}

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}





