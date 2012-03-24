#ifndef __NODEACTION_H__
#define __NODEACTION_H__

#include "simulation/environment.h"
#include "simulation/basicobjects.h"

// an action that moves a node
// completely specified by ActionDesc
class NodeMoveAction : public Action {
    btSoftBody *psb;
    Environment::Ptr env;

    SphereObject::Ptr anchorpt; // an object that we can attach an an anchor to
    int anchoridx;
    bool anchorAppended;
    BulletObject::MoveAction::Ptr moveaction;

    void appendAnchor(const btTransform &trans);
    void removeAnchor();

public:
    struct Spec {
        int i; // index of node on cloth
        btVector3 v; // direction to move the node, magnitude signifies distance
        float t; // execution time for the action
        float movingFrac;
        Spec() { }
        Spec(int i_, const btVector3 &v_, float t_) : i(i_), v(v_), t(t_), movingFrac(0.05) { }
    } spec;

    NodeMoveAction() : anchorAppended(false) { }

    // sets the environment and soft body to act on
    void setSoftBody(Environment::Ptr env_, btSoftBody *psb_) {
        env = env_; psb = psb_;
    }

    void readSpec() { setExecTime(spec.t); }

    void reset() {
        Action::reset();
        removeAnchor();
    }

    void step(float dt);
};

// a fake list
// do NOT use two actions returned by at() at once! they will be
// the same NodeMoveAction object, just with different specs
class NodeActionList {
    NodeMoveAction ac;
    vector<NodeMoveAction::Spec> specs;

public:
    NodeActionList() { }

    void add(const NodeMoveAction::Spec &s) { specs.push_back(s); }
    void clear() { specs.clear(); }
    int size() const { return specs.size(); }

    NodeMoveAction &at(int i) {
        ac.reset();
        ac.spec = specs[i];
        ac.readSpec();
        return ac;
    }
    NodeMoveAction &operator[](int i) { return at(i); }
};


// utility methods for filling NodeActionLists with pre-set actions
void genCompleteSpecsForNode(NodeActionList &l, int idx, bool excludeNoOp=true);
void genSpecsForNodeWithoutCorners(NodeActionList &l, int idx);
void genSpecsForCloth(NodeActionList &l, int resx, int resy);


#endif // __NODEACTION_H__
