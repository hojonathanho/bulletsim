#ifndef __FL_EDGES_H__
#define __FL_EDGES_H__

#include "cloth.h"

// out is populated with node indices that could serve as grasp edges
void calcFoldNodes(const Cloth &cloth, vector<int> &out);

btVector3 calcFoldLineDir(const Cloth &cloth, int node, const vector<int> &foldnodes, bool zeroZ=true);

#endif // __FL_EDGES_H__
