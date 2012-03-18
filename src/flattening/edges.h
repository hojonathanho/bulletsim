#ifndef __FL_EDGES_H__
#define __FL_EDGES_H__

#include "clothutil.h"

// out is populated with node indices that could serve as grasp edges
void calcFoldNodes(const ClothSpec &cs, vector<int> &out);

btVector3 calcFoldLineDir(const ClothSpec &cs, int node, const vector<int> &foldnodes, bool zeroZ=true);

#endif // __FL_EDGES_H__
