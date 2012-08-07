#pragma once
#include "environment.h"
#include "basicobjects.h"
#include "btBulletDynamicsCommon.h"
#include <vector>

class Finger : public CompoundObject<BulletObject> {
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
  typedef boost::shared_ptr<Finger> Ptr;
  std::vector<BulletConstraint::Ptr> m_joints;
  float m_linDamping;
  float m_angDamping;
  Finger(btScalar mass, vector<float> radii, vector<float> heights, const btTransform &initTrans, float linDamping=0.05, float angDamping=0.85);
  void init();
};

class Thumb : public CompoundObject<BulletObject> {
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
	typedef boost::shared_ptr<Thumb> Ptr;
  std::vector<BulletConstraint::Ptr> m_joints;
  float m_linDamping;
  float m_angDamping;
	Thumb(btScalar mass, vector<float> radii, vector<float> heights, const btTransform &initTrans, float linDamping=0.05, float angDamping=0.85);
  void init();
};

class Palm : public CompoundObject<BulletObject> {
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
	typedef boost::shared_ptr<Palm> Ptr;
  std::vector<BulletConstraint::Ptr> m_joints;
  float m_linDamping;
  float m_angDamping;
	Palm(btScalar mass, vector<float> radii, vector<float> heights, const btTransform &initTrans, float linDamping=0.05, float angDamping=0.85);
  void init();
};

class Hand : public CompoundObject<CompoundObject<BulletObject> > {
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

public:
	typedef boost::shared_ptr<Hand> Ptr;
  std::vector<BulletConstraint::Ptr> m_joints;
  float m_linDamping;
  float m_angDamping;
	Hand(btScalar mass, vector<vector<float> > radii, vector<vector<float> > heights, const btTransform &initTrans, float linDamping=0.05, float angDamping=0.85);
	Hand(btScalar mass, const btTransform &initTrans, float linDamping=0.05, float angDamping=0.85);
  void init();
};

Hand::Ptr makeHand(btScalar mass, const btTransform &initTrans, float linDamping=0.05, float angDamping=0.85);
