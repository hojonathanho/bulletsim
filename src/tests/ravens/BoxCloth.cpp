#include "BoxCloth.h"
#include <iostream>
#include <simulation/bullet_io.h>
#include <boost/foreach.hpp>
#include "utils/conversions.h"
#include "utils/utils_vector.h"
#include <utility>

using namespace std;


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

/*
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
		for (int i=0; i<=5; i++) {
			holder[k]->enableSpring(i,true);
			holder[k]->setStiffness(i,stiffness);
			holder[k]->setDamping(i,damping);
		}
		holder[k]->setAngularLowerLimit(btVector3(-limit,-limit,-limit));
		holder[k]->setAngularUpperLimit(btVector3(limit,limit,limit));
	}
	return holder;
}*/


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

		holder[k].reset(new btGeneric6DofSpringConstraint(*rbA,*rbB,tA,tB,true));
		for (int i=0; i<=5; i++) {
			holder[k]->enableSpring(i,true);
			holder[k]->setStiffness(i,0);
			holder[k]->setDamping(i,0);
		}



		//holder[k]->setAngularLowerLimit(btVector3(-limit,-limit,-limit));
		//holder[k]->setAngularUpperLimit(btVector3(limit,limit,limit));
		holder[k]->setEquilibriumPoint();

	}
	return holder;
}


bool BoxCloth::isHole(unsigned int i, unsigned int j) {
	int k = 0;
	while (k < hole_is.size() && !(hole_is[k] == i && hole_js[k] == j)) {
		k ++;
	}
	return !(k == hole_is.size());
}


unsigned int BoxCloth::getSerializedIndex(unsigned int i, unsigned int j) {
	return (i*m + j);
}


BoxCloth::BoxCloth(unsigned int n_, unsigned int m_, vector<unsigned int> hole_is_, vector<unsigned int> hole_js_,
		btScalar s_, btScalar h_, float angStiffness_, float linDamping_,
		float angDamping_, float angLimit_) : n(n_), m(m_), s(s_*METERS), h(h_*METERS),
		hole_is(hole_is_), hole_js(hole_js_)     {

	angStiffness = angStiffness_;
	angDamping   = angDamping_;
	angLimit     = angLimit_;
	linDamping   = linDamping_;

	assert(("BoxCloth: The indices for holes should have the same size!" , hole_is.size() == hole_js.size()));

	vector<btTransform> transforms;
	vector< vector<BoxObject> >  boxes;

	// height above the ground
	const btScalar vertical_offset = 1*METERS;

	createBoxTransforms(transforms,n,m,s, btVector3(0,0,vertical_offset));

	btVector3 halfExtents(s/2, s/2, h/2);
	btScalar  mass = 1000.0;

	// create boxes.
	for (unsigned int i=0; i < n; i++) {
		for(unsigned int j=0; j<m; j++) {
			if (! isHole(i,j)) {
				btTransform trans = transforms[getSerializedIndex(i,j)];
				BoxObject::Ptr child(new BoxObject(mass, halfExtents, trans));
				child->rigidBody->setDamping(linDamping, angDamping);
				child->rigidBody->setFriction(1);
				children.push_back(child);
				grid_to_obj_inds.insert(make_pair(make_pair(i,j), children.size()-1));

			}
		}
	}

	// create constraints

	// corner constraints
	unsigned int num_holes = hole_is.size();
	unsigned int corners_x[] = {0, 0 , n-1, n-1};
	unsigned int corners_y[] = {0, m-1 , 0, m-1};
	for (int i = 0; i < 4; i ++) {
		boost::shared_ptr<btPoint2PointConstraint> jointPtr(new btPoint2PointConstraint(*children[grid_to_obj_inds[make_pair(corners_x[i], corners_y[i])]]->rigidBody,btVector3(0,0,0)));
		joints.push_back(BulletConstraint::Ptr(new BulletConstraint(jointPtr, true)));
	}
	children[grid_to_obj_inds[make_pair(corners_x[0], corners_y[0])]]->setColor(1,0,0,0.8);
	children[grid_to_obj_inds[make_pair(corners_x[1], corners_y[1])]]->setColor(0,1,0,0.8);
	children[grid_to_obj_inds[make_pair(corners_x[2], corners_y[2])]]->setColor(0,0,1,0.8);
	children[grid_to_obj_inds[make_pair(corners_x[3], corners_y[3])]]->setColor(1,0,1,0.8);

	for (unsigned int i=0; i < n; i++) {
		for(unsigned int j=0; j<m; j++) {
			if (! isHole(i,j)) {
				unsigned int curr  = grid_to_obj_inds[make_pair(i,j)];
				unsigned int right = grid_to_obj_inds[make_pair(i+1,j)];
				unsigned int below = grid_to_obj_inds[make_pair(i,j+1)];

				if (!isHole(i+1,j) && getSerializedIndex(i+1,j) >= 0 && getSerializedIndex(i+1,j) < n*m) {
					vector<boost::shared_ptr<btGeneric6DofSpringConstraint> > springPtrs = createBendConstraint2(s, true, children[curr]->rigidBody,children[right]->rigidBody,angDamping,angStiffness,angLimit);
					for(int k=0; k < springPtrs.size(); k+=1)
						joints.push_back(BulletConstraint::Ptr(new BulletConstraint(springPtrs[k], true)));
				}

				if (!isHole(i,j+1) && getSerializedIndex(i,j+1) >= 0 && getSerializedIndex(i,j+1) < n*m && j < m-1)  {
					vector<boost::shared_ptr<btGeneric6DofSpringConstraint> > springPtrs = createBendConstraint2(s, false, children[curr]->rigidBody,children[below]->rigidBody,angDamping,angStiffness,angLimit);
					for(int k=0; k < springPtrs.size(); k+=1)
						joints.push_back(BulletConstraint::Ptr(new BulletConstraint(springPtrs[k], true)));
				}
			}
		}
	}

}



void BoxCloth::init() {
	CompoundObject<BulletObject>::init();
	for (int i=0; i< joints.size(); i++)
		getEnvironment()->addConstraint(joints[i]);
}

void BoxCloth::destroy() {
	CompoundObject<BulletObject>::destroy();
}
