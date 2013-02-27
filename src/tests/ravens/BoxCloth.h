#pragma once
#include "environment.h"
#include "basicobjects.h"
#include "btBulletDynamicsCommon.h"
#include <vector>
#include <utils/config.h>

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

	// spring parameters
	float angStiffness;
	float linDamping;
	float angDamping;
	float angLimit;

public:
	typedef boost::shared_ptr<BoxCloth> Ptr;
	std::vector<boost::shared_ptr<btCollisionShape> > shapes;
	std::vector<BulletConstraint::Ptr> joints;

	BoxCloth(unsigned int n_, unsigned int m_, btScalar s_=METERS*0.02, btScalar h_=METERS*0.00, float angStiffness_=.1, float linDamping_=.75, float angDamping_=1, float angLimit_=.4);

	void init();
	void destroy();
	void setTexture(cv::Mat image, cv::Mat mask, const btTransform& camFromWorld);
};
