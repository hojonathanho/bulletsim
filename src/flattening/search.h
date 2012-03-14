#ifndef __FLATTENING_SEARCH_H__
#define __FLATTENING_SEARCH_H__

#include "nodeactions.h"
#include "simulation/softbodies.h"

class Scene;
class NodeActionList;

//NodeMoveAction::Spec flattenCloth_greedy_single(const Environment::Ptr env, const SoftObject::Ptr cloth,
//        const vector<NodeMoveAction::Spec> &candspecs);
void flattenCloth_greedy(Scene &scene, BulletSoftObject::Ptr initCloth,
        NodeActionList &candidateActions, int steps, NodeActionList &out, bool actOnInputEnv=true);

#endif //  __FLATTENING_SEARCH_H__
