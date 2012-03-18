#include "edges.h"

static const btScalar distSqIfFlat(const ClothSpec &cs, int n1, int n2) {
    int x1 = CLOTHIDX_X(cs, n1);
    int y1 = CLOTHIDX_Y(cs, n1);
    int x2 = CLOTHIDX_X(cs, n2);
    int y2 = CLOTHIDX_Y(cs, n2);
    btScalar dx = (x1 - x2) * cs.lenx/cs.resx;
    btScalar dy = (y1 - y2) * cs.leny/cs.resy;
    return dx*dx + dy*dy;
}

// (x0, y0) and (x1, y1) are two points on the cloth
// this checks whether the two points, in relation to the given center pt,
// constitute a bent line
static const btScalar BEND_ANGLE = M_PI/2;
static bool isBent(const ClothSpec &cs, const btVector3 &center,
        int x0, int y0, int x1, int y1) {
    if (CLOTHCOORDS_OOB(cs, x0, y0) || CLOTHCOORDS_OOB(cs, x1, y1)) return false;
    int i0 = CLOTHIDX(cs, x0, y0);
    int i1 = CLOTHIDX(cs, x1, y1);
    btVector3 v0 = cs.psb->m_nodes[i0].m_x - center;
    btVector3 v1 = cs.psb->m_nodes[i1].m_x - center;
    btScalar angle = v0.angle(v1);
    // the angle is 0 degrees for fully bent, 180 degrees for not bent
    return angle < BEND_ANGLE;
}

//static const btScalar FLATDIST_SCALE = 1.2;
void calcDiscontNodes(const ClothSpec &cs, vector<int> &out) {
    // get "usual" node distance (as if the cloth were flat)
    const btScalar flatdist = max(cs.lenx/cs.resx, cs.leny/cs.resy);
    for (int i = 0; i < cs.psb->m_nodes.size(); ++i) {
        const btSoftBody::Node &n = cs.psb->m_nodes[i];
        int x = CLOTHIDX_X(cs, i), y = CLOTHIDX_Y(cs, i);

        // if this is an edge node, then add this to out
        if (i < CLOTHIDX(cs, 0, 1)
                || (i >= CLOTHIDX(cs, 0, cs.resy-1)
                    && i <= CLOTHIDX(cs, cs.resx-1, cs.resy-1))
                || x == 0
                || x == cs.resx-1) {
            out.push_back(i);
            continue;
        }

        // check if the cloth is bent at the current node
        // by looking at nodes distance 2 away
        const int d = 2;
        if (isBent(cs, n.m_x, x-d, y, x+d, y)
                || isBent(cs, n.m_x, x, y-d, x, y+d)
                || isBent(cs, n.m_x, x-d, y-d, x+d, y+d)
                || isBent(cs, n.m_x, x+d, y-d, x-d, y+d))
            out.push_back(i);
    }
}
