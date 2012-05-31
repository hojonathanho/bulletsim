#ifndef __FL_CLOTH_H__
#define __FL_CLOTH_H__

#include "simulation/softbodies.h"
#include "flattening/make_bodies.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

struct Cloth : public BulletSoftObject {
    typedef boost::shared_ptr<Cloth> Ptr;

    const int resx, resy;
    const btScalar lenx, leny;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::KdTree<pcl::PointXYZ>::Ptr kdtree;

    Cloth(int resx_, int resy_,
          btScalar lenx_, btScalar leny_,
          const btVector3 &initCenter,
          btSoftBodyWorldInfo &worldInfo);
    Cloth(int resx_, int resy_,
          btScalar lenx_, btScalar leny_,
          BulletSoftObject::Ptr sb);

    EnvironmentObject::Ptr copy(Fork &f) const;

    void updateAccel(); // update acceleration structures (kdtree, etc)
    btSoftBody *psb() const { return softBody.get(); }
    // (x,y) coord to nodeidx conversions and tests
    int idx(int x, int y) const { return y * resx + x; }
    int coordX(int idx) const { return idx % resx; }
    int coordY(int idx) const { return idx / resx; }
    bool coordsOOB(int x, int y) const {
        return x < 0 || x >= resx || y < 0 || y >= resy;
    }
    bool idxOOB(int i) const { return coordsOOB(coordX(i), coordY(i)); }
    bool idxOnTopEdge(int i) const { return i < idx(0, 1); }
    bool idxOnBottomEdge(int i) const { return i >= idx(0, resy-1) && i <= idx(resx-1, resy-1); }
    bool idxOnLeftEdge(int i) const { return coordX(i) == 0; }
    bool idxOnRightEdge(int i) const { return coordX(i) == resx-1; }
    bool idxOnEdge(int i) const { return idxOnTopEdge(i) || idxOnBottomEdge(i) || idxOnLeftEdge(i) || idxOnRightEdge(i); }

    // serialization
    static Ptr createFromFile(btSoftBodyWorldInfo& worldInfo, const char* fileName);
    static Ptr createFromFile(btSoftBodyWorldInfo& worldInfo, const string &fileName) { return createFromFile(worldInfo, fileName.c_str()); }
    static Ptr createFromFile(btSoftBodyWorldInfo& worldInfo, istream &s);
    void saveToFile(const char *fileName) const;
    void saveToFile(const string &fileName) const { saveToFile(fileName.c_str()); }
    void saveToFile(ostream &s) const;

    // utility functions

    // calculates center of the cloth
    btVector3 centerPoint() const;

    // applies rigid transformation relative to the given point
    void translateRel(const btTransform &t, const btVector3 &pt);
    void translateRelToCenter(const btTransform &t) { translateRel(t, centerPoint()); }

    // translates the cloth so that its center lies at the given point
    void translateCenterToPt(const btVector3 &pt);

private:
    void initAccel();
};

#endif // __FL_CLOTH_H__
