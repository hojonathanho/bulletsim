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

        // otherwise, see if the map from current cloth state -> flat cloth state
        // is discontinuous at the current node (looking at 4 adjacent nodes)
 /*       pcl::PointXYZ searchpt(n.m_x.x(), n.m_x.y(), n.m_x.z());
        vector<int> pointIdxRadiusSearch;
        vector<float> pointRadiusSquaredDistance;
        if (cs.kdtree->radiusSearch(searchpt, flatdist,
                    pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            cout << "nn gives " << pointRadiusSquaredDistance.size() << " nodes" << endl;
            for (int z = 0; z < pointIdxRadiusSearch.size(); ++z) {
                int tmpidx = pointIdxRadiusSearch[z];
                if (distSqIfFlat(cs, i, tmpidx) > flatdist*flatdist) {
                    cout << "WUT" << endl;
                    out.push_back(tmpidx);
                }
            }
        }*/
        /*int checkX[] = { x-1, x+1,   x,   x };
        int checkY[] = {   y,   y, y-1, y+1 };
        for (int z = 0; z < 4; ++z) {
            if (checkX[z] < 0 || checkX[z] >= cs.resx
                    || checkY[z] < 0 || checkY[z] >= cs.resy)
                continue;
            int newidx = CLOTHIDX(cs, checkX[z], checkY[z]);
            if (n.m_x.distance(cs.psb->m_nodes[newidx].m_x) > flatdist)
                out.push_back(i);
        }*/
    }
}
