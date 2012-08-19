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

btSoftBody*	CreateFromSoftBodyExcludeNodes(btSoftBody* softBody, std::vector<int> exclude_nodes_idx);
btSoftBody*	CreateFromSoftBodyExcludeFaces(btSoftBody* softBody, std::vector<int> exclude_faces_idx);

btSoftBody* CreatePolygonPatch(btSoftBodyWorldInfo& worldInfo, std::vector<btVector3> corners, int resx, int resy, bool gendiags);

#endif /* SOFTBODYHELPERS_H_ */
