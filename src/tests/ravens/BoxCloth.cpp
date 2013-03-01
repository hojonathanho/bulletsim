#include "BoxCloth.h"
#include <iostream>
#include <simulation/bullet_io.h>
#include <boost/foreach.hpp>
#include "utils/conversions.h"
#include "utils/utils_vector.h"
#include <utility>

using namespace std;


/** Useful for creating rigid bodies: creates collision shape, motion state etc.*/
boost::shared_ptr<btRigidBody> createRigidBody(const boost::shared_ptr<btCollisionShape> shapePtr, const btTransform& trans, btScalar mass) {
	bool isDynamic = (mass != 0.f);
	btVector3 localInertia(0,0,0);
	if (isDynamic) shapePtr->calculateLocalInertia(mass,localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shapePtr.get(),localInertia);
	return boost::shared_ptr<btRigidBody>(new btRigidBody(cInfo));
}

/** Creates a matrix of transformation for each of the n x m boxes.
 *  The side length along x,y dimensions is assumed to be s.
 *  The transforms are centered around CENTER. */
void createBoxTransforms(vector<btTransform> &transforms, unsigned int n, unsigned int m, btScalar s, btVector3 center) {
	transforms.clear();
	for(unsigned int i=0; i<n; i += 1) {
		for (unsigned int j=0; j<m; j += 1) {
			btTransform T;
			T.setIdentity();
			btVector3 loc(-1*((n/2)*s)+(i*s), -1*((m/2)*s)+(j*s), 0.0);
			loc += center;
			T.setOrigin(loc);
			transforms.push_back(T);
		}
	}
}


vector<boost::shared_ptr<btGeneric6DofSpringConstraint> >
createBendConstraint(btScalar side_len, bool isX,
		               const boost::shared_ptr<btRigidBody> rbA,
		               const boost::shared_ptr<btRigidBody>& rbB,
		               float damping, float stiffness, float limit) {


	vector<btVector3> offsetA, offsetB;
	if (isX) {
		offsetA.push_back(btVector3(side_len/2,  0, 0));
		offsetB.push_back(btVector3(-side_len/2, 0, 0));
	} else {
		offsetA.push_back(btVector3(0, -side_len/2,0));
		offsetB.push_back(btVector3(0,  side_len/2,0));
	}


	boost::shared_ptr<btGeneric6DofSpringConstraint> springPtr1;
	vector<boost::shared_ptr<btGeneric6DofSpringConstraint> > holder;
	holder.push_back(springPtr1);

	for (int k=0; k<1; k++) {
		btTransform tA,tB;
		tA.setIdentity(); tB.setIdentity();
		tA.setOrigin(offsetA[k]);
		tB.setOrigin(offsetB[k]);

		holder[k].reset(new btGeneric6DofSpringConstraint(*rbA,*rbB,tA,tB,false));
		for (int i=3; i<=5; i++) {
			holder[k]->enableSpring(i,true);
			holder[k]->setStiffness(i,stiffness);
			holder[k]->setDamping(i,damping);
		}
		holder[k]->setAngularLowerLimit(btVector3(-limit,-limit,-limit));
		holder[k]->setAngularUpperLimit(btVector3(limit,limit,limit));
	}
	return holder;
}


vector<boost::shared_ptr<btGeneric6DofSpringConstraint> >
createBendConstraint2(btScalar side_len, bool isX,
		               const boost::shared_ptr<btRigidBody> rbA,
		               const boost::shared_ptr<btRigidBody>& rbB,
		               float damping, float stiffness, float limit) {


	vector<btVector3> offsetA, offsetB;
	if (isX) {
		offsetA.push_back(btVector3(side_len/2,  side_len/2.01, 0));
		offsetA.push_back(btVector3(side_len/2, -side_len/2.01, 0));
		offsetB.push_back(btVector3(-side_len/2, side_len/2.01, 0));
		offsetB.push_back(btVector3(-side_len/2,-side_len/2.01, 0));
	} else {
		offsetA.push_back(btVector3(side_len/2.01, -side_len/2,0));
		offsetA.push_back(btVector3(-side_len/2.01,-side_len/2,0));
		offsetB.push_back(btVector3(side_len/2.01,  side_len/2,0));
		offsetB.push_back(btVector3(-side_len/2.01, side_len/2,0));
	}


	boost::shared_ptr<btGeneric6DofSpringConstraint> springPtr1;
	boost::shared_ptr<btGeneric6DofSpringConstraint> springPtr2;
	vector<boost::shared_ptr<btGeneric6DofSpringConstraint> > holder;
	holder.push_back(springPtr1);
	holder.push_back(springPtr2);

	for (int k=0; k<2; k++) {
		btTransform tA,tB;
		tA.setIdentity(); tB.setIdentity();
		tA.setOrigin(offsetA[k]);
		tB.setOrigin(offsetB[k]);

		holder[k].reset(new btGeneric6DofSpringConstraint(*rbA,*rbB,tA,tB,false));
		for (int i=3; i<=5; i++) {
			holder[k]->enableSpring(i,true);
			holder[k]->setStiffness(i,stiffness);
			holder[k]->setDamping(i,damping);
		}
		holder[k]->setAngularLowerLimit(btVector3(-limit,-limit,-limit));
		holder[k]->setAngularUpperLimit(btVector3(limit,limit,limit));
	}
	return holder;
}

BoxCloth::BoxCloth(unsigned int n_, unsigned int m_, btScalar s_, btScalar h_,
					 float angStiffness_, float linDamping_, float angDamping_, float angLimit_) : n(n_), m(m_), s(s_*METERS), h(h_*METERS) {
	angStiffness = angStiffness_;
	angDamping   = angDamping_;
	angLimit     = angLimit_;
	linDamping   = linDamping_;

	vector<btTransform> transforms;
	vector< vector<BoxObject> >  boxes;

	btScalar vertical_offset = 1*METERS;


	createBoxTransforms(transforms,n,m,s, btVector3(0,0,vertical_offset));
	btVector3 halfExtents(s/2, s/2, h/2);
	btScalar  mass = 100.0;

	// create boxes.
	for (unsigned int i=0; i < n; i++) {
		for(unsigned int j=0; j<m; j++) {
			btTransform trans = transforms[i*m+j];
			BoxObject::Ptr child(new BoxObject(mass, halfExtents, trans));
			child->rigidBody->setDamping(linDamping, angDamping);
			child->rigidBody->setFriction(1);
			children.push_back(child);
		}
	}

	// create constraints

	boost::shared_ptr<btPoint2PointConstraint> jointPtr1A(new btPoint2PointConstraint(*children[0]->rigidBody,btVector3(0,0,0)));
	joints.push_back(BulletConstraint::Ptr(new BulletConstraint(jointPtr1A, true)));

	boost::shared_ptr<btPoint2PointConstraint> jointPtr2(new btPoint2PointConstraint(*children[m-1]->rigidBody,btVector3(0,0,0)));
	joints.push_back(BulletConstraint::Ptr(new BulletConstraint(jointPtr2, true)));

	boost::shared_ptr<btPoint2PointConstraint> jointPtr3(new btPoint2PointConstraint(*children[m*n-m]->rigidBody,btVector3(0,0,0)));
	joints.push_back(BulletConstraint::Ptr(new BulletConstraint(jointPtr3, true)));

	boost::shared_ptr<btPoint2PointConstraint> jointPtr4A(new btPoint2PointConstraint(*children[n*m-1]->rigidBody,btVector3(0,0,0)));
	joints.push_back(BulletConstraint::Ptr(new BulletConstraint(jointPtr4A, true)));



	for (unsigned int i=0; i < n; i++) {
		for(unsigned int j=0; j<m; j++) {
			unsigned int curr  = (i*m) + j;
			unsigned int right = curr + m;
			unsigned int below = curr + 1;

			if (right >= 0 && right < n*m) {
				vector<boost::shared_ptr<btGeneric6DofSpringConstraint> > springPtrs = createBendConstraint2(s, true, children[curr]->rigidBody,children[right]->rigidBody,angDamping,angStiffness,angLimit);
				for(int k=0; k < springPtrs.size(); k+=1)
					joints.push_back(BulletConstraint::Ptr(new BulletConstraint(springPtrs[k], true)));
			}

			if (below >= 0 && below < n*m && j < m-1)  {
				vector<boost::shared_ptr<btGeneric6DofSpringConstraint> > springPtrs = createBendConstraint2(s, false, children[curr]->rigidBody,children[below]->rigidBody,angDamping,angStiffness,angLimit);
				for(int k=0; k < springPtrs.size(); k+=1)
					joints.push_back(BulletConstraint::Ptr(new BulletConstraint(springPtrs[k], true)));

			}
		}
	}

		children[0]->setColor(1,0,0,0.8);
	children[m-1]->setColor(0,1,0,0.8);
	children[n*m-m]->setColor(0,0,1,0.8);
	children[n*m-1]->setColor(1,0,1,0.8);
}


void BoxCloth::init() {
	CompoundObject<BulletObject>::init();
	cout<<"num joints: "<<joints.size()<<endl;
	for (int i=0; i< joints.size(); i++) {
		getEnvironment()->addConstraint(joints[i]);
	}
}

void BoxCloth::destroy() {
	CompoundObject<BulletObject>::destroy();
}
