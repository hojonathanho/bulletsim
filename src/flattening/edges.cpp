#include "edges.h"
#include <pcl/common/pca.h>

static btScalar distSqIfFlat(const Cloth &cloth, int n1, int n2) {
    int x1 = cloth.coordX(n1);
    int y1 = cloth.coordY(n1);
    int x2 = cloth.coordX(n2);
    int y2 = cloth.coordY(n2);
    btScalar dx = (x1 - x2) * cloth.lenx/cloth.resx;
    btScalar dy = (y1 - y2) * cloth.leny/cloth.resy;
    return dx*dx + dy*dy;
}

// (x0, y0) and (x1, y1) are two points on the cloth
// this checks whether the two points, in relation to the given center pt,
// constitute a bent line
static const btScalar BEND_ANGLE = M_PI/2;
static bool isBent(const Cloth &cloth, const btVector3 &center,
        int x0, int y0, int x1, int y1) {
    if (cloth.coordsOOB(x0, y0) || cloth.coordsOOB(x1, y1)) return false;
    int i0 = cloth.idx(x0, y0);
    int i1 = cloth.idx(x1, y1);
    btVector3 v0 = cloth.psb()->m_nodes[i0].m_x - center;
    btVector3 v1 = cloth.psb()->m_nodes[i1].m_x - center;
    btScalar angle = v0.angle(v1);
    // the angle is 0 degrees for fully bent, 180 degrees for not bent
    return angle < BEND_ANGLE;
}

//static const btScalar FLATDIST_SCALE = 1.2;
void calcFoldNodes(const Cloth &cloth, vector<int> &out) {
    // get "usual" node distance (as if the cloth were flat)
    const btScalar flatdist = max(cloth.lenx/cloth.resx, cloth.leny/cloth.resy);
    for (int i = 0; i < cloth.psb()->m_nodes.size(); ++i) {
        const btSoftBody::Node &n = cloth.psb()->m_nodes[i];
        int x = cloth.coordX(i), y = cloth.coordY(i);

        // if this is an edge node, then add this to out
        if (cloth.idxOnTopEdge(i)
                || cloth.idxOnBottomEdge(i)
                || cloth.idxOnLeftEdge(i)
                || cloth.idxOnRightEdge(i)) {
            out.push_back(i);
            continue;
        }

        // check if the cloth is bent at the current node
        // by looking at nodes distance 2 away
        static const int d = 2;
        if (isBent(cloth, n.m_x, x-d, y, x+d, y)
                || isBent(cloth, n.m_x, x, y-d, x, y+d)
                || isBent(cloth, n.m_x, x-d, y-d, x+d, y+d)
                || isBent(cloth, n.m_x, x+d, y-d, x-d, y+d))
            out.push_back(i);
    }
}

btVector3 calcFoldLineDir(const Cloth &cloth, int node, const vector<int> &foldnodes, bool zeroZ) {
    // if the node is an edge node, then give the direction of the edge
    int othernode = -1;
    if (cloth.idxOnTopEdge(node)
            || cloth.idxOnBottomEdge(node)) {
        if (cloth.coordX(node) < cloth.resx - 1)
            othernode = node + 1;
        else
            othernode = node - 1;
    } else if (cloth.idxOnLeftEdge(node)
            || cloth.idxOnRightEdge(node)) {
        if (cloth.coordY(node) < cloth.resy - 1)
            othernode = node + cloth.resx;
        else
            othernode = node - cloth.resx;
    }
    if (othernode != -1)
        return (cloth.psb()->m_nodes[node].m_x
                - cloth.psb()->m_nodes[othernode].m_x).normalized();

    // otherwise:
    // get a bunch of nodes in some neighborhood of the given node
    vector<int> nearnodes;
    btScalar searchradius = 2*max(cloth.lenx/cloth.resx, cloth.leny/cloth.resy);
    const btVector3 &p = cloth.psb()->m_nodes[node].m_x;
    pcl::PointXYZ searchpt(p.x(), p.y(), p.z());
    vector<int> pointIdxRadiusSearch;
    vector<float> pointRadiusSquaredDistance;
    if (cloth.kdtree->radiusSearch(
                searchpt, searchradius,
                pointIdxRadiusSearch,
                pointRadiusSquaredDistance) > 0) {
        for (int i = 0; i < pointIdxRadiusSearch.size(); ++i)
            nearnodes.push_back(pointIdxRadiusSearch[i]);
    }

    // not enough nodes for pca
    if (nearnodes.size() < 3)
        return btVector3(0, 0, 0);

    // return the first principal component of the nearnodes
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize(nearnodes.size());
    for (int i = 0; i < cloud->points.size(); ++i) {
        const btVector3 &p = cloth.psb()->m_nodes[nearnodes[i]].m_x;
        cloud->points[i].x = p.x();
        cloud->points[i].y = p.y();
        cloud->points[i].z = zeroZ ? 0 : p.z();
    }
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    Eigen::VectorXf comp0 = pca.getEigenVectors().col(0);
    return btVector3(comp0(0), comp0(1), comp0(2));
}

btVector3 calcGraspDir(const Cloth &cloth, int node) {
    // if the node is an edge node, then point straight out
    int othernode = -1;
    if (cloth.idxOnTopEdge(node))
        othernode = node + cloth.resx;
    else if (cloth.idxOnBottomEdge(node))
        othernode = node - cloth.resx;
    else if (cloth.idxOnLeftEdge(node))
        othernode = node + 1;
    else if (cloth.idxOnRightEdge(node))
        othernode = node - 1;
    if (othernode != -1 && !cloth.idxOOB(othernode))
        return (cloth.psb()->m_nodes[othernode].m_x
                - cloth.psb()->m_nodes[node].m_x).normalized();

    // if not an edge, just return the normal
    return cloth.psb()->m_nodes[node].m_n;
}
