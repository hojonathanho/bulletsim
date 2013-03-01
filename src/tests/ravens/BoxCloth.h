#pragma once
#include <simulation/environment.h>
#include <simulation/basicobjects.h>
#include "btBulletDynamicsCommon.h"
#include <vector>
#include <utils/config.h>
#include <utility>

class BoxCloth : public CompoundObject<BulletObject> {

private:
	// number of boxes along the x-dimension
	int n;

	// number of boxes along the y-dimension
	int m;

	// size of the side of the box objects along x and y dimensions
	btScalar s;

	// height of the box-objects along the z dimension
	btScalar h;

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

public:
	typedef boost::shared_ptr<BoxCloth> Ptr;
	std::vector<boost::shared_ptr<btCollisionShape> > shapes;
	std::vector<BulletConstraint::Ptr> joints;

	BoxCloth(unsigned int n_, unsigned int m_, vector<unsigned int> hole_is_, vector<unsigned int> hole_js_,
			  btScalar s_=METERS*0.02, btScalar h_=METERS*0.001, float angStiffness_=0,
			  float linDamping_=0.3, float angDamping_=0.1, float angLimit_=1);

	void init();
	void destroy();
};
