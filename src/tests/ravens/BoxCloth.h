#pragma once
#include <simulation/environment.h>
#include <simulation/basicobjects.h>
#include "btBulletDynamicsCommon.h"
#include <vector>
#include <utils/config.h>
#include <utility>


class BoxCloth : public CompoundObject<BulletObject> {

public:
	// number of boxes along the x-dimension
	int n;

	// number of boxes along the y-dimension
	int m;

	// size of the side of the box objects along x and y dimensions
	btScalar s;

	// height of the box-objects along the z dimension
	btScalar h;

	// center of the cloth
	btVector3 center;

	//indices of the squares in the 2d grid where holes should be located.
	vector<unsigned int> hole_is, hole_js;

	// grid(i,j) |--> index of square object in this->children
	std::map<std::pair<unsigned int, unsigned int>, unsigned int> grid_to_obj_inds;

	// spring/ constraints parameters
	float angStiffness;
	float linDamping;
	float angDamping;
	float angLimit;


	bool isHole(unsigned int i, unsigned int j);
	unsigned int getSerializedIndex(unsigned int i, unsigned int j);

	typedef boost::shared_ptr<BoxCloth> Ptr;
	std::vector<boost::shared_ptr<btCollisionShape> > shapes;
	std::vector<CompoundObject<BulletObject>::Ptr>    holes;
	std::vector<BulletConstraint::Ptr> joints;

	BoxCloth(unsigned int n_, unsigned int m_, vector<unsigned int> hole_is_, vector<unsigned int> hole_js_,
			  btScalar s_=METERS*0.02, btScalar h_=METERS*0.001, btVector3 center_=btVector3(0,0,0),
			  float angStiffness_=1e5, float linDamping_=0.3, float angDamping_=0.1, float angLimit_=0.5);


	void makeHole (btTransform &tfm);
	void addChild (float mass, btVector3 hfExtents, btTransform &tfm);
	void addHoleConstraint (vector<btVector3> &offsetA, vector<btVector3> &offsetB,
	  	  	  				const boost::shared_ptr<btRigidBody> rbA,
	  	  	  				const boost::shared_ptr<btRigidBody>& rbB);

	string objectType () {return "BoxCloth";}

	void getBoxClothPoints(vector< pair<int, int> > indices, vector<btVector3> & boxPoints);
	void getBoxClothHoles(vector<btVector3> & holePoints);

	void init();
	void destroy();
};
