/*
 * softBodyHelpers.h
 *
 *  Created on: Aug 18, 2012
 *      Author: alex
 */

#ifndef SOFTBODYHELPERS_H_
#define SOFTBODYHELPERS_H_

#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <vector>

//DEBUG
#include "simulation/environment.h"


/** Structure to specify a plane which cuts a softbody.*/
struct cutPlane {
	std::vector<btVector3> 	c_corners;
	btVector3				c_origin,c_xax,c_yax,c_zax;
	btScalar 				c_f1,c_f2,c_length;

	cutPlane (btVector3 p1, btVector3 p2, btVector3 p3, btVector3 p4,
			  btScalar f1, btScalar f2)	{
		assert(0<f1 && f1<f2 && f2<1);
		c_f1 = f1;
		c_f2 = f2;
		c_corners.push_back(p1);
		c_corners.push_back(p2);
		c_corners.push_back(p3);
		c_corners.push_back(p4);
		c_xax = (p3-p2).normalized();
		c_yax = (p3-p4).normalized();
		c_zax = c_xax.cross(c_yax).normalized();

		c_origin = 0.5*(p2+p1) + f1*(p4-p1);
		c_length = (f2-f1)*(p4-p1).length();
	}

	bool shouldConsider (const btVector3& p) {
		btScalar x = (p-c_origin).dot(c_xax);
		return 0 <= x && x <= c_length;
	}

	btScalar Eval(btVector3& p) {
		return (p-c_origin).dot(c_yax);
	}
};


/** Solves for the scalar t: info->EVal(c) = 0, where c = ta + (1-t)b : 0<=t<=1.*/
static btScalar	ImplicitSolve( cutPlane* inf,
							   btVector3& a,
							   btVector3& b,
							   const btScalar accuracy,
							   const int maxiterations=256);


/** Cuts the PSB based on cutting plane information in INFO.
 *  Cuts are executed within ACCURACY of the specifications. */
void cutPlaneSoftBody(btSoftBody* psb, cutPlane* info, btScalar accuracy);


btSoftBody*	CreateFromSoftBodyExcludeNodes(btSoftBody* softBody, std::vector<int> exclude_nodes_idx);
btSoftBody*	CreateFromSoftBodyExcludeFaces(btSoftBody* softBody, std::vector<int> exclude_faces_idx);

btSoftBody* CreatePolygonPatch(btSoftBodyWorldInfo& worldInfo, std::vector<btVector3> corners, int resx, int resy, bool gendiags);


#endif /* SOFTBODYHELPERS_H_ */
