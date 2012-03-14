#include "nodeactions.h"
#include "simulation/config_bullet.h"

void NodeMoveAction::appendAnchor(const btTransform &trans) {
    if (anchorAppended) return;

    anchorpt.reset(new SphereObject(0, 0.005*METERS, trans, true));
    anchorpt->setColor(1, 0, 1, 1);
    anchorpt->rigidBody->setCollisionFlags(anchorpt->rigidBody->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
    env->add(anchorpt);
    moveaction = anchorpt->createMoveAction();
    moveaction->setExecTime(execTime * spec.movingFrac);

    psb->appendAnchor(spec.i, anchorpt->rigidBody.get());
    anchoridx = psb->m_anchors.size() - 1;
    if (anchoridx != 0) {
        cout << "WARNING: attaching multiple anchors to cloth in NodeMoveAction!" << endl;
        BOOST_ASSERT(false);
    }

    anchorAppended = true;
}

void NodeMoveAction::removeAnchor() {
    if (!anchorAppended) return;

    env->remove(anchorpt);
    anchorpt.reset();
    moveaction.reset();

    psb->m_anchors[anchoridx].m_node->m_battach = 0;
    if (psb->m_anchors.size() != 1) {
        cout << "WARNING: number of anchors on cloth != 1" << endl;
        BOOST_ASSERT(false);
    }
    psb->m_anchors.clear();

    anchorAppended = false;
}

void NodeMoveAction::step(float dt) {
    if (done()) return;

    if (timeElapsed == 0) {
        // first step
        const btVector3 &nodepos = psb->m_nodes[spec.i].m_x;
        btTransform starttrans(btQuaternion(0, 0, 0, 1), nodepos);
        btTransform endtrans(btQuaternion(0, 0, 0, 1), nodepos + spec.v);
        appendAnchor(starttrans);
        moveaction->setEndpoints(starttrans, endtrans);
    }

    moveaction->step(dt);
    stepTime(dt);

    if (done()) removeAnchor();
}

// utility methods for filling NodeActionLists with pre-set actions
void genCompleteSpecsForNode(NodeActionList &l, int idx, bool excludeNoOp) {
    static float ACTION_TIME = 0.5;
    static btScalar d = 0.5*METERS;
    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j)
            for (int k = -1; k <= 1; ++k)
                if (!excludeNoOp || i != 0 || j != 0 || k != 0)
                    l.add(NodeMoveAction::Spec(idx, d * btVector3(i, j, k).normalized(), ACTION_TIME));
}
void genSpecsForNodeWithoutCorners(NodeActionList &l, int idx) {
    static float ACTION_TIME = 0.5;
    static btScalar d = 0.5*METERS;

    l.add(NodeMoveAction::Spec(idx, d * btVector3(1, 0, 0), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, 1, 0), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, 0, 1), ACTION_TIME));

    l.add(NodeMoveAction::Spec(idx, d * btVector3(-1, 0, 0), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, -1, 0), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, 0, -1), ACTION_TIME));

    l.add(NodeMoveAction::Spec(idx, d * btVector3(1, 1, 0).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(-1, 1, 0).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(-1, -1, 0).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(1, -1, 0).normalized(), ACTION_TIME));

    l.add(NodeMoveAction::Spec(idx, d * btVector3(1, 0, 1).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(-1, 0, 1).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(-1, 0, -1).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(1, 0, -1).normalized(), ACTION_TIME));

    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, 1, 1).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, -1, 1).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, -1, -1).normalized(), ACTION_TIME));
    l.add(NodeMoveAction::Spec(idx, d * btVector3(0, 1, -1).normalized(), ACTION_TIME));
}

void genSpecsForCloth(NodeActionList &l, int resx, int resy) {
#define IDX(_x_, _y_) ((_y_)*resx+(_x_))

    // just the 4 corners for now
    genCompleteSpecsForNode(l, IDX(0, 0));
    genCompleteSpecsForNode(l, IDX(resx-1, 0));
    genCompleteSpecsForNode(l, IDX(0, resy-1));
    genCompleteSpecsForNode(l, IDX(resx-1, resy-1));

#undef IDX
}
