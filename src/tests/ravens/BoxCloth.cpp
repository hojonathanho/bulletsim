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

/*
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
 */
vector<boost::shared_ptr<btGeneric6DofSpringConstraint> >
createBendConstraint(  btScalar side_len,
		vector< boost::shared_ptr<btRigidBody> > & rbA,
		vector< boost::shared_ptr<btRigidBody> > & rbB,
		vector<btVector3> & offsetA,
		vector<btVector3> & offsetB,
		float damping, float stiffness, float limit) {

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

		holder[k].reset(new btGeneric6DofSpringConstraint(*rbA[k],*rbB[k],tA,tB,true));
		for (int i=0; i<=5; i++) {
			holder[k]->enableSpring(i,true);
			holder[k]->setStiffness(i,0);
			holder[k]->setDamping(i,0);
		}
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


// Assuming no hole is a corner.
void BoxCloth::makeHole(btTransform &tfm){
	// Create the 4 box objects
	btVector3 halfExtents1(s/6, s/6, h/2);
	btScalar  mass1 = 100.0;
	btVector3 halfExtents2(s/2, s/6, h/2);
	btScalar  mass2 = 300.0;

	unsigned int size = children.size();

	btTransform tfm1(tfm);
	tfm1.getOrigin() += s/3*tfm1.getBasis().getColumn(0);
	addChild (mass1, halfExtents1, tfm1);

	btTransform tfm2(tfm);
	tfm2.getOrigin() -= s/3*tfm2.getBasis().getColumn(0);
	addChild (mass1, halfExtents1, tfm2);

	btTransform tfm3(tfm);
	tfm3.getOrigin() += s/3*tfm3.getBasis().getColumn(1);
	addChild (mass2, halfExtents2, tfm3);

	btTransform tfm4(tfm);
	tfm4.getOrigin() -= s/3*tfm4.getBasis().getColumn(1);
	addChild (mass2, halfExtents2, tfm4);

	// Adding constraints
	vector<btVector3> offsetA, offsetB;

	offsetA.push_back(btVector3(s/12,s/6,0));
	offsetA.push_back(btVector3(-s/12,s/6,0));
	offsetB.push_back(btVector3(5*s/12,-s/6,0));
	offsetB.push_back(btVector3(3*s/12,-s/6,0));
	addHoleConstraint(offsetA, offsetB, children[size]->rigidBody, children[size+2]->rigidBody);

	offsetA.clear(); offsetB.clear();
	offsetA.push_back(btVector3(s/12,-s/6,0));
	offsetA.push_back(btVector3(-s/12,-s/6,0));
	offsetB.push_back(btVector3(5*s/12,s/6,0));
	offsetB.push_back(btVector3(3*s/12,s/6,0));
	addHoleConstraint(offsetA, offsetB, children[size]->rigidBody, children[size+3]->rigidBody);

	offsetA.clear(); offsetB.clear();
	offsetA.push_back(btVector3(s/12,s/6,0));
	offsetA.push_back(btVector3(-s/12,s/6,0));
	offsetB.push_back(btVector3(-3*s/12,-s/6,0));
	offsetB.push_back(btVector3(-5*s/12,-s/6,0));
	addHoleConstraint(offsetA, offsetB, children[size+1]->rigidBody, children[size+2]->rigidBody);

	offsetA.clear(); offsetB.clear();
	offsetA.push_back(btVector3(s/12,-s/6,0));
	offsetA.push_back(btVector3(-s/12,-s/6,0));
	offsetB.push_back(btVector3(-3*s/12,s/6,0));
	offsetB.push_back(btVector3(-5*s/12,s/6,0));
	addHoleConstraint(offsetA, offsetB, children[size+1]->rigidBody, children[size+3]->rigidBody);
}

void BoxCloth::addChild (float mass, btVector3 hfExtents, btTransform &tfm) {
	BoxObject::Ptr child(new BoxObject(mass, hfExtents, tfm));
	child->rigidBody->setDamping(linDamping, angDamping);
	child->rigidBody->setFriction(1);
	children.push_back(child);
}

void BoxCloth::addHoleConstraint (vector<btVector3> &offsetA, vector<btVector3> &offsetB,
		const boost::shared_ptr<btRigidBody> rbA,
		const boost::shared_ptr<btRigidBody>& rbB) {

	for (int k=0; k<2; k++) {
		boost::shared_ptr<btGeneric6DofConstraint> constraintPtr;

		btTransform tA,tB;
		tA.setIdentity(); tB.setIdentity();
		tA.setOrigin(offsetA[k]);
		tB.setOrigin(offsetB[k]);

		constraintPtr.reset(new btGeneric6DofConstraint(*rbA,*rbB,tA,tB,true));
		constraintPtr->setLinearLowerLimit(btVector3(0,0,0));
		constraintPtr->setLinearUpperLimit(btVector3(0,0,0));
		constraintPtr->setAngularLowerLimit(btVector3(0,0,0));
		constraintPtr->setAngularUpperLimit(btVector3(0,0,0));

		joints.push_back(BulletConstraint::Ptr(new BulletConstraint(constraintPtr, true)));
	}
}

BoxCloth::BoxCloth(unsigned int n_, unsigned int m_, vector<unsigned int> hole_is_, vector<unsigned int> hole_js_,
		btScalar s_, btScalar h_, btVector3 center_, float angStiffness_, float linDamping_,
		float angDamping_, float angLimit_) : n(n_), m(m_), s(s_*METERS), h(h_*METERS), center (center_*METERS),
		hole_is(hole_is_), hole_js(hole_js_)     {

	angStiffness = angStiffness_;
	angDamping   = angDamping_;
	angLimit     = angLimit_;
	linDamping   = linDamping_;

	assert(("BoxCloth: The indices for holes should have the same size!" , hole_is.size() == hole_js.size()));

	vector<btTransform> transforms;
	vector< vector<BoxObject> >  boxes;

	createBoxTransforms(transforms,n,m,s, center);

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
			else {
				btTransform trans = transforms[getSerializedIndex(i,j)];
				makeHole(trans);
				grid_to_obj_inds.insert(make_pair(make_pair(i,j), children.size()-4));
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

			unsigned int curr  = grid_to_obj_inds[make_pair(i,j)];
			unsigned int right = grid_to_obj_inds[make_pair(i+1,j)];
			unsigned int below = grid_to_obj_inds[make_pair(i,j+1)];

			if (getSerializedIndex(i+1,j) >= 0 && getSerializedIndex(i+1,j) < n*m) {

				vector<boost::shared_ptr<btGeneric6DofSpringConstraint> > springPtrs;
				vector< boost::shared_ptr<btRigidBody> > rbA, rbB;
				vector<btVector3> offsetA, offsetB;

				if (!isHole(i,j)) {
					rbA.push_back(children[curr]->rigidBody); rbA.push_back(children[curr]->rigidBody);
					offsetA.push_back(btVector3 (s/2,s/2.01,0)); offsetA.push_back(btVector3 (s/2,-s/2.01,0));
				} else {
					rbA.push_back(children[curr+2]->rigidBody); rbA.push_back(children[curr+3]->rigidBody);
					offsetA.push_back(btVector3 (s/2,s/2.01-s/3,0)); offsetA.push_back(btVector3 (s/2,-s/2.01+s/3,0));
				}

				if (!isHole(i+1,j)) {
					rbB.push_back(children[right]->rigidBody); rbB.push_back(children[right]->rigidBody);
					offsetB.push_back(btVector3 (-s/2,s/2.01,0)); offsetB.push_back(btVector3 (-s/2,-s/2.01,0));
				} else {
					rbB.push_back(children[right+2]->rigidBody); rbB.push_back(children[right+3]->rigidBody);
					offsetB.push_back(btVector3 (-s/2,s/2.01-s/3,0)); offsetB.push_back(btVector3 (-s/2,-s/2.01+s/3,0));
				}

				springPtrs = createBendConstraint(s, rbA, rbB, offsetA, offsetB, angDamping, angStiffness, angLimit);
				for(int k=0; k < springPtrs.size(); k+=1)
					joints.push_back(BulletConstraint::Ptr(new BulletConstraint(springPtrs[k], true)));
			}

			if (getSerializedIndex(i,j+1) >= 0 && getSerializedIndex(i,j+1) < n*m && j < m-1)  {

				vector<boost::shared_ptr<btGeneric6DofSpringConstraint> > springPtrs;
				vector< boost::shared_ptr<btRigidBody> > rbA, rbB;
				vector<btVector3> offsetA, offsetB;

				if (!isHole(i,j)) {
					rbA.push_back(children[curr]->rigidBody); rbA.push_back(children[curr]->rigidBody);
					offsetA.push_back(btVector3 (s/2.01,-s/2,0)); offsetA.push_back(btVector3 (-s/2.01,-s/2,0));
				} else {
					rbA.push_back(children[curr+3]->rigidBody); rbA.push_back(children[curr+3]->rigidBody);
					offsetA.push_back(btVector3 (s/2.01,-s/6,0)); offsetA.push_back(btVector3 (-s/2.01,-s/6,0));
				}

				if (!isHole(i,j+1)) {
					rbB.push_back(children[below]->rigidBody); rbB.push_back(children[below]->rigidBody);
					offsetB.push_back(btVector3 (s/2.01,s/2,0)); offsetB.push_back(btVector3 (-s/2.01,s/2,0));
				}else {
					rbB.push_back(children[below+2]->rigidBody); rbB.push_back(children[below+2]->rigidBody);
					offsetB.push_back(btVector3 (s/2.01,s/6,0)); offsetB.push_back(btVector3 (-s/2.01,s/6,0));
				}

				springPtrs = createBendConstraint(s, rbA, rbB, offsetA, offsetB, angDamping, angStiffness, angLimit);
				for(int k=0; k < springPtrs.size(); k+=1)
					joints.push_back(BulletConstraint::Ptr(new BulletConstraint(springPtrs[k], true)));

			}
		}
	}
}

/** Returns vector of points uniformly sampled across each non-hole box-cloth. */
void BoxCloth::getBoxClothPoints(vector< pair<int, int> > indices, vector<btVector3> & boxPoints) {

	for (unsigned int i=0; i < indices.size(); ++i) {
		if (getSerializedIndex(indices[i].first+1,indices[i].second) >= 0 &&
				getSerializedIndex(indices[i].first+1,indices[i].second) < n*m &&
				!isHole(indices[i].first,indices[i].second)) {

			unsigned int curr = grid_to_obj_inds[indices[i]];
			btTransform currTfm = children[curr]->getIndexTransform(0);
			btVector3 center = currTfm.getOrigin();
			btVector3 xVec = currTfm.getBasis().getColumn(0), yVec = currTfm.getBasis().getColumn(1);

			boxPoints.push_back(center);
			boxPoints.push_back(center+xVec*s/3+yVec*s/3);
			boxPoints.push_back(center+xVec*s/3-yVec*s/3);
			boxPoints.push_back(center-xVec*s/3+yVec*s/3);
			boxPoints.push_back(center-xVec*s/3-yVec*s/3);
			boxPoints.push_back(center+xVec*s/4+yVec*s/4);
			boxPoints.push_back(center+xVec*s/4-yVec*s/4);
			boxPoints.push_back(center-xVec*s/4+yVec*s/4);
			boxPoints.push_back(center-xVec*s/4-yVec*s/4);
		}
	}
}

/** Returns vector of points uniformly sampled across each non-hole box-cloth. */
void BoxCloth::getBoxClothHoles(vector<btVector3> & holePoints) {

	for (unsigned int k=0; k < hole_is.size(); k++) {
		unsigned int curr = grid_to_obj_inds[make_pair(hole_is[k],hole_js[k])];

		btTransform topTfm = children[curr+2]->getIndexTransform(0);
		btTransform botTfm = children[curr+3]->getIndexTransform(0);
		btVector3 xVec = topTfm.getBasis().getColumn(0), yVec = topTfm.getBasis().getColumn(1);
		btVector3 center = topTfm.getOrigin() - yVec*s/3;

		holePoints.push_back(children[curr]->getIndexTransform(0).getOrigin());
		holePoints.push_back(children[curr+1]->getIndexTransform(0).getOrigin());
		holePoints.push_back(topTfm.getOrigin() + xVec*s/3);
		holePoints.push_back(topTfm.getOrigin() - xVec*s/3);
		holePoints.push_back(botTfm.getOrigin() + xVec*s/3);
		holePoints.push_back(botTfm.getOrigin() - xVec*s/3);
		holePoints.push_back(center);
		holePoints.push_back(center+xVec*s/6+yVec*s/6);
		holePoints.push_back(center+xVec*s/6-yVec*s/6);
		holePoints.push_back(center-xVec*s/6+yVec*s/6);
		holePoints.push_back(center-xVec*s/6-yVec*s/6);
		holePoints.push_back(center+xVec*s/8+yVec*s/8);
		holePoints.push_back(center+xVec*s/8-yVec*s/8);
		holePoints.push_back(center-xVec*s/8+yVec*s/8);
		holePoints.push_back(center-xVec*s/8-yVec*s/8);
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
