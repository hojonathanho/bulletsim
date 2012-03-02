#ifndef __FLATTENING_SEARCH_H__
#define __FLATTENING_SEARCH_H__

#include "nodeactions.h"
#include "simulation/softbodies.h"

class Scene;
class NodeActionList;

void flattenCloth_greedy(Scene &scene, BulletSoftObject::Ptr initCloth,
        NodeActionList &candidateActions, int steps, NodeActionList &out);

#endif //  __FLATTENING_SEARCH_H__
