#include "hand.h"
#include "basicobjects.h"
#include <iostream>
#include "bullet_io.h"
#include "utils/config.h"
using namespace std;

void HumanHandObject::MyNearCallback(btBroadphasePair& collisionPair,
	btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo) {

	// Do your collision logic here
	void* other_client = NULL;
	if (children[0]->rigidBody.get() == collisionPair.m_pProxy0->m_clientObject)
		other_client = collisionPair.m_pProxy1->m_clientObject;
	else if (children[0]->rigidBody.get() == collisionPair.m_pProxy1->m_clientObject)
		other_client = collisionPair.m_pProxy0->m_clientObject;
	if (other_client)
		if ((children[2]->rigidBody.get() == other_client) ||
				(children[6]->rigidBody.get() == other_client) ||
				(children[10]->rigidBody.get() == other_client) ||
				(children[14]->rigidBody.get() == other_client) ||
				(children[18]->rigidBody.get() == other_client))
			return;

	// Only dispatch the Bullet collision information if you want the physics to continue
	dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
}

HumanHandObject::HumanHandObject(RaveInstance::Ptr rave) :
	RaveObject(rave, EXPAND(BULLETSIM_DATA_DIR)"/robot_model/HumanHand/HumanHand20DOF.robot.xml", CONVEX_HULL, false, true) {
	setColor(239.0/255.0, 208.0/255.0, 207.0/255.0, 0.6);
}

void HumanHandObject::init() {
	RaveObject::init();

	// set callback for ignoring collisions between the proximal phalanges and the palm
	boost::function<void(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo)>
		near_cb = boost::bind(&HumanHandObject::MyNearCallback, this, _1, _2, _3);
	getEnvironment()->bullet->dispatcher->setNearCallback(*near_cb.target<btNearCallback>());

	vector<KinBody::JointPtr> vbodyjoints; vbodyjoints.reserve(body->GetJoints().size()+body->GetPassiveJoints().size());
	vbodyjoints.insert(vbodyjoints.end(),body->GetJoints().begin(),body->GetJoints().end());
	vbodyjoints.insert(vbodyjoints.end(),body->GetPassiveJoints().begin(),body->GetPassiveJoints().end());
	BOOST_FOREACH(KinBody::JointPtr joint, vbodyjoints) {
		m_constraints.push_back(jointMap[joint]);
	}
}

vector<float> HumanHandObject::getJointAngles() {
	vector<float> angles;
	for (int i=0; i<m_constraints.size(); i++)
		angles.push_back(boost::dynamic_pointer_cast<btHingeConstraint>(m_constraints[i]->cnt)->getHingeAngle());
	return angles;
}



Finger::Finger (btScalar mass, vector<float> radii, vector<float> heights, const btTransform &initTrans, float linDamping, float angDamping)
	: m_linDamping(linDamping),
	  m_angDamping(angDamping)
{
	assert(radii.size() == PART_COUNT);
	assert(heights.size() == PART_COUNT);

	children.resize(PART_COUNT);
	m_joints.resize(JOINT_COUNT);

	// Setup all the capsule objects
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_PROXIMAL]/2.0), btScalar(0.)));
	children[PART_PROXIMAL] = CapsuleObjectY::Ptr(new CapsuleObjectY(mass/3.0, radii[PART_PROXIMAL], heights[PART_PROXIMAL],initTrans*transform));

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_PROXIMAL] - heights[PART_MIDDLE]/2.0), btScalar(0.)));
	children[PART_MIDDLE] = CapsuleObjectY::Ptr(new CapsuleObjectY(mass/3.0, radii[PART_MIDDLE], heights[PART_MIDDLE],initTrans*transform));

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_PROXIMAL] - heights[PART_MIDDLE] - heights[PART_DISTAL]/2.0), btScalar(0.)));
	children[PART_DISTAL] = CapsuleObjectY::Ptr(new CapsuleObjectY(mass/3.0, radii[PART_DISTAL], heights[PART_DISTAL],initTrans*transform));

	// Setup some damping on the rigid bodies
	for (int i = 0; i < PART_COUNT; ++i)
	{
		children[i]->rigidBody->setDamping(m_linDamping, m_angDamping);
		children[i]->rigidBody->setDeactivationTime(0.8);
		children[i]->rigidBody->setSleepingThresholds(1.6, 2.5);
	}

	// Now setup the constraints
	btHingeConstraint* hingeC;

	btTransform localA, localB;

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_PROXIMAL]/2.0), btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(heights[PART_MIDDLE]/2.0), btScalar(0.)));
	hingeC =  new btHingeConstraint(*children[PART_PROXIMAL]->rigidBody, *children[PART_MIDDLE]->rigidBody, localA, localB);
	hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
	m_joints[JOINT_PROXIMAL] = BulletConstraint::Ptr(new BulletConstraint(boost::shared_ptr<btHingeConstraint>(hingeC), true));

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_MIDDLE]/2.0), btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(heights[PART_DISTAL]/2.0), btScalar(0.)));
	hingeC =  new btHingeConstraint(*children[PART_MIDDLE]->rigidBody, *children[PART_DISTAL]->rigidBody, localA, localB);
	hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
	m_joints[JOINT_DISTAL] = BulletConstraint::Ptr(new BulletConstraint(boost::shared_ptr<btHingeConstraint>(hingeC), true));
}

void Finger::init() {
  CompoundObject<BulletObject>::init();
  for (int i=0; i< m_joints.size(); i++) {
    getEnvironment()->addConstraint(m_joints[i]);
  }
}

Thumb::Thumb (btScalar mass, vector<float> radii, vector<float> heights, const btTransform &initTrans, float linDamping, float angDamping)
	: m_linDamping(linDamping),
	  m_angDamping(angDamping)
{
	assert(radii.size() == PART_COUNT);
	assert(heights.size() == PART_COUNT);

	children.resize(PART_COUNT);
	m_joints.resize(JOINT_COUNT);

	// Setup all the capsule objects
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_PROXIMAL]/2.0), btScalar(0.)));
	children[PART_PROXIMAL] = CapsuleObjectY::Ptr(new CapsuleObjectY(mass/2.0, radii[PART_PROXIMAL], heights[PART_PROXIMAL],initTrans*transform));

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_PROXIMAL] - heights[PART_DISTAL]/2.0), btScalar(0.)));
	children[PART_DISTAL] = CapsuleObjectY::Ptr(new CapsuleObjectY(mass/2.0, radii[PART_DISTAL], heights[PART_DISTAL],initTrans*transform));

	// Setup some damping on the rigid bodies
	for (int i = 0; i < PART_COUNT; ++i)
	{
		children[i]->rigidBody->setDamping(m_linDamping, m_angDamping);
		children[i]->rigidBody->setDeactivationTime(0.8);
		children[i]->rigidBody->setSleepingThresholds(1.6, 2.5);
	}

	// Now setup the constraints
	btHingeConstraint* hingeC;

	btTransform localA, localB;

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-heights[PART_PROXIMAL]/2.0), btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(heights[PART_DISTAL]/2.0), btScalar(0.)));
	hingeC =  new btHingeConstraint(*children[PART_PROXIMAL]->rigidBody, *children[PART_DISTAL]->rigidBody, localA, localB);
	hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
	m_joints[JOINT_PROXIMAL] = BulletConstraint::Ptr(new BulletConstraint(boost::shared_ptr<btHingeConstraint>(hingeC), true));
}

void Thumb::init() {
  CompoundObject<BulletObject>::init();
  for (int i=0; i< m_joints.size(); i++) {
    getEnvironment()->addConstraint(m_joints[i]);
  }
}


Palm::Palm (btScalar mass, vector<float> radii, vector<float> heights, const btTransform &initTrans, float linDamping, float angDamping)
	: m_linDamping(linDamping),
	  m_angDamping(angDamping)
{
	assert(radii.size() == PART_COUNT);
	assert(heights.size() == PART_COUNT);

	children.resize(PART_COUNT);
	m_joints.resize(JOINT_COUNT);

	// Setup all the capsule objects
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-radii[PART_THUMB] - 2.0*radii[PART_INDEX] - radii[PART_MIDDLE]), -heights[PART_THUMB]/2.0, btScalar(0.)));
	children[PART_THUMB] = CapsuleObjectY::Ptr(new CapsuleObjectY(mass/5.0, radii[PART_THUMB], heights[PART_THUMB],initTrans*transform));

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-radii[PART_INDEX] - radii[PART_MIDDLE]), -heights[PART_INDEX]/2.0, btScalar(0.)));
	children[PART_INDEX] = CapsuleObjectY::Ptr(new CapsuleObjectY(mass/5.0, radii[PART_INDEX], heights[PART_INDEX],initTrans*transform));

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.), -heights[PART_MIDDLE]/2.0, btScalar(0.)));
	children[PART_MIDDLE] = CapsuleObjectY::Ptr(new CapsuleObjectY(mass/5.0, radii[PART_MIDDLE], heights[PART_MIDDLE],initTrans*transform));

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(radii[PART_INDEX] + radii[PART_RING]), -heights[PART_RING]/2.0, btScalar(0.)));
	children[PART_RING] = CapsuleObjectY::Ptr(new CapsuleObjectY(mass/5.0, radii[PART_RING], heights[PART_RING],initTrans*transform));

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(radii[PART_INDEX] + 2.0*radii[PART_RING] + radii[PART_LITTLE]), -heights[PART_LITTLE]/2.0, btScalar(0.)));
	children[PART_LITTLE] = CapsuleObjectY::Ptr(new CapsuleObjectY(mass/5.0, radii[PART_LITTLE], heights[PART_LITTLE],initTrans*transform));

	// Setup some damping on the rigid bodies
	for (int i = 0; i < PART_COUNT; ++i)
	{
		children[i]->rigidBody->setDamping(m_linDamping, m_angDamping);
		children[i]->rigidBody->setDeactivationTime(0.8);
		children[i]->rigidBody->setSleepingThresholds(1.6, 2.5);
	}

	// Now setup the constraints
	btHingeConstraint* hingeC;

	btTransform localA, localB;

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localA.setOrigin(btVector3(btScalar(radii[PART_THUMB]), heights[PART_THUMB]/2.0, btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localB.setOrigin(btVector3(btScalar(-radii[PART_INDEX]), heights[PART_INDEX]/2.0, btScalar(0.)));
	hingeC =  new btHingeConstraint(*children[PART_THUMB]->rigidBody, *children[PART_INDEX]->rigidBody, localA, localB);
	hingeC->setLimit(btScalar(0), btScalar(M_PI/8.0));
	m_joints[JOINT_THUMB_INDEX] = BulletConstraint::Ptr(new BulletConstraint(boost::shared_ptr<btHingeConstraint>(hingeC), true));

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localA.setOrigin(btVector3(btScalar(radii[PART_INDEX]), heights[PART_INDEX]/2.0, btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localB.setOrigin(btVector3(btScalar(-radii[PART_MIDDLE]), heights[PART_MIDDLE]/2.0, btScalar(0.)));
	hingeC =  new btHingeConstraint(*children[PART_INDEX]->rigidBody, *children[PART_MIDDLE]->rigidBody, localA, localB);
	hingeC->setLimit(btScalar(0), btScalar(M_PI/8.0));
	m_joints[JOINT_INDEX_MIDDLE] = BulletConstraint::Ptr(new BulletConstraint(boost::shared_ptr<btHingeConstraint>(hingeC), true));

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localA.setOrigin(btVector3(btScalar(radii[PART_MIDDLE]), heights[PART_MIDDLE]/2.0, btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localB.setOrigin(btVector3(btScalar(-radii[PART_RING]), heights[PART_RING]/2.0, btScalar(0.)));
	hingeC =  new btHingeConstraint(*children[PART_MIDDLE]->rigidBody, *children[PART_RING]->rigidBody, localA, localB);
	hingeC->setLimit(btScalar(0), btScalar(M_PI/8.0));
	m_joints[JOINT_MIDDLE_RING] = BulletConstraint::Ptr(new BulletConstraint(boost::shared_ptr<btHingeConstraint>(hingeC), true));

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localA.setOrigin(btVector3(btScalar(radii[PART_RING]), heights[PART_RING]/2.0, btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,M_PI_2); localB.setOrigin(btVector3(btScalar(-radii[PART_LITTLE]), heights[PART_LITTLE]/2.0, btScalar(0.)));
	hingeC =  new btHingeConstraint(*children[PART_RING]->rigidBody, *children[PART_LITTLE]->rigidBody, localA, localB);
	hingeC->setLimit(btScalar(0), btScalar(M_PI/8.0));
	m_joints[JOINT_RING_LITTLE] = BulletConstraint::Ptr(new BulletConstraint(boost::shared_ptr<btHingeConstraint>(hingeC), true));
}

void Palm::init() {
  CompoundObject<BulletObject>::init();
  for (int i=0; i< m_joints.size(); i++) {
    getEnvironment()->addConstraint(m_joints[i]);
  }
}

Hand::Hand(btScalar mass, vector<vector<float> > radii, vector<vector<float> > heights, const btTransform &initTrans, float linDamping, float angDamping)
	: m_linDamping(linDamping),
		m_angDamping(angDamping)
{
	assert(radii.size() == PART_COUNT);
	assert(heights.size() == PART_COUNT);

	children2.resize(PART_COUNT);
	m_joints.resize(JOINT_COUNT);

	// Setup all the hand parts
	btTransform transform;
	transform.setIdentity();
	children2[PART_PALM]   = Palm::Ptr(new Palm(mass*5.0/19.0, radii[PART_PALM], heights[PART_PALM], initTrans*transform, linDamping, angDamping));

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-radii[Hand::PART_PALM][Palm::PART_THUMB] - 2.0*radii[Hand::PART_PALM][Palm::PART_INDEX] - radii[Hand::PART_PALM][Palm::PART_MIDDLE]),
			 btScalar(-heights[Hand::PART_PALM][Palm::PART_THUMB]),
			 btScalar(0.)));
	transform.getBasis().setEulerZYX(0,0,-M_PI/8.0);
	children2[PART_THUMB]  = Thumb::Ptr(new Thumb(mass*2.0/19.0, radii[PART_THUMB], heights[PART_THUMB], initTrans*transform, linDamping, angDamping));

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(-radii[Hand::PART_PALM][Palm::PART_INDEX] - radii[Hand::PART_PALM][Palm::PART_MIDDLE]),
			 btScalar(-heights[Hand::PART_PALM][Palm::PART_INDEX]),
			 btScalar(0.)));
	children2[PART_INDEX]  = Finger::Ptr(new Finger(mass*3.0/19.0, radii[PART_INDEX], heights[PART_INDEX], initTrans*transform, linDamping, angDamping));

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.),
			 btScalar(-heights[Hand::PART_PALM][Palm::PART_MIDDLE]),
			 btScalar(0.)));
	children2[PART_MIDDLE] = Finger::Ptr(new Finger(mass*3.0/19.0, radii[PART_MIDDLE], heights[PART_MIDDLE], initTrans*transform, linDamping, angDamping));

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(radii[Hand::PART_PALM][Palm::PART_MIDDLE] + radii[Hand::PART_PALM][Palm::PART_RING]),
			 btScalar(-heights[Hand::PART_PALM][Palm::PART_RING]),
			 btScalar(0.)));
	children2[PART_RING]   = Finger::Ptr(new Finger(mass*3.0/19.0, radii[PART_RING], heights[PART_RING], initTrans*transform, linDamping, angDamping));

	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(radii[Hand::PART_PALM][Palm::PART_MIDDLE] + 2.0*radii[Hand::PART_PALM][Palm::PART_RING] + radii[Hand::PART_PALM][Palm::PART_LITTLE]),
			 btScalar(-heights[Hand::PART_PALM][Palm::PART_LITTLE]),
			 btScalar(0.)));
	children2[PART_LITTLE] = Finger::Ptr(new Finger(mass*3.0/19.0, radii[PART_LITTLE], heights[PART_LITTLE], initTrans*transform, linDamping, angDamping));

	// Now setup the constraints
	btHingeConstraint* hingeC;

	btTransform localA, localB;

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,-M_PI/8.0); localA.setOrigin(btVector3(btScalar(0.), -heights[Hand::PART_PALM][Palm::PART_THUMB]/2.0, btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), heights[Hand::PART_THUMB][Thumb::PART_PROXIMAL]/2.0, btScalar(0.)));
	btPoint2PointConstraint* ptp =  new btPoint2PointConstraint(*children2[Hand::PART_PALM]->children[Palm::PART_THUMB]->rigidBody, *children2[Hand::PART_THUMB]->children[Thumb::PART_PROXIMAL]->rigidBody, localA.getOrigin(), localB.getOrigin());
//	hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
	m_joints[JOINT_THUMB_KNUCKLES] = BulletConstraint::Ptr(new BulletConstraint(boost::shared_ptr<btPoint2PointConstraint>(ptp), true));

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), -heights[Hand::PART_PALM][Palm::PART_INDEX]/2.0, btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), heights[Hand::PART_INDEX][Finger::PART_PROXIMAL]/2.0, btScalar(0.)));
	hingeC =  new btHingeConstraint(*children2[Hand::PART_PALM]->children[Palm::PART_INDEX]->rigidBody, *children2[Hand::PART_INDEX]->children[Finger::PART_PROXIMAL]->rigidBody, localA, localB);
	hingeC->setLimit(btScalar(-M_PI/8.0), btScalar(M_PI_2));
	m_joints[JOINT_INDEX_KNUCKLES] = BulletConstraint::Ptr(new BulletConstraint(boost::shared_ptr<btHingeConstraint>(hingeC), true));

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), -heights[Hand::PART_PALM][Palm::PART_MIDDLE]/2.0, btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), heights[Hand::PART_MIDDLE][Finger::PART_PROXIMAL]/2.0, btScalar(0.)));
	hingeC =  new btHingeConstraint(*children2[Hand::PART_PALM]->children[Palm::PART_MIDDLE]->rigidBody, *children2[Hand::PART_MIDDLE]->children[Finger::PART_PROXIMAL]->rigidBody, localA, localB);
	hingeC->setLimit(btScalar(-M_PI/8.0), btScalar(M_PI_2));
	m_joints[JOINT_MIDDLE_KNUCKLES] = BulletConstraint::Ptr(new BulletConstraint(boost::shared_ptr<btHingeConstraint>(hingeC), true));

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), -heights[Hand::PART_PALM][Palm::PART_RING]/2.0, btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), heights[Hand::PART_RING][Finger::PART_PROXIMAL]/2.0, btScalar(0.)));
	hingeC =  new btHingeConstraint(*children2[Hand::PART_PALM]->children[Palm::PART_RING]->rigidBody, *children2[Hand::PART_RING]->children[Finger::PART_PROXIMAL]->rigidBody, localA, localB);
	hingeC->setLimit(btScalar(-M_PI/8.0), btScalar(M_PI_2));
	m_joints[JOINT_RING_KNUCKLES] = BulletConstraint::Ptr(new BulletConstraint(boost::shared_ptr<btHingeConstraint>(hingeC), true));

	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), -heights[Hand::PART_PALM][Palm::PART_LITTLE]/2.0, btScalar(0.)));
	localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), heights[Hand::PART_LITTLE][Finger::PART_PROXIMAL]/2.0, btScalar(0.)));
	hingeC =  new btHingeConstraint(*children2[Hand::PART_PALM]->children[Palm::PART_LITTLE]->rigidBody, *children2[Hand::PART_LITTLE]->children[Finger::PART_PROXIMAL]->rigidBody, localA, localB);
	hingeC->setLimit(btScalar(-M_PI/8.0), btScalar(M_PI_2));
	m_joints[JOINT_LITTLE_KNUCKLES] = BulletConstraint::Ptr(new BulletConstraint(boost::shared_ptr<btHingeConstraint>(hingeC), true));

	for (int i=0; i < children2.size(); ++i)
		for (int j=0; j < children2[i]->children.size(); j++)
			children.push_back(children2[i]->children[j]);

}

void Hand::init() {
//  CompoundObject<BulletObject>::init();
  for (int i=0; i < children2.size(); ++i) {
	  children2[i]->setEnvironment(getEnvironment());
	  children2[i]->init();
  }
  for (int i=0; i< m_joints.size(); i++) {
	  getEnvironment()->addConstraint(m_joints[i]);
  }
}

Hand::Ptr makeHand(btScalar mass, const btTransform &initTrans, float linDamping, float angDamping) {
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

	return Hand::Ptr(new Hand(mass, radii, heights, initTrans, linDamping, angDamping));
}

