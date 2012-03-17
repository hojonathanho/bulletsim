#ifndef __FL_EDGES_H__
#define __FL_EDGES_H__

#include "clothutil.h"

// out is populated with node indices that could serve as grasp edges
void calcDiscontNodes(const ClothSpec &cs, vector<int> &out);

#endif // __FL_EDGES_H__
