#ifndef __CONVEXDECOMP_H__
#define __CONVEXDECOMP_H__

#include <hacdHACD.h>
#include <btBulletDynamicsCommon.h>
#include <boost/shared_ptr.hpp>
#include <vector>

class ConvexDecomp {
private:
    const float margin;

    HACD::Vec3<HACD::Real> toHACDVec(const btVector3 &v) {
        return HACD::Vec3<HACD::Real>(v.x(), v.y(), v.z());
    }

    btVector3 toBtVector(const HACD::Vec3<HACD::Real> &v) {
        return btVector3(v.X(), v.Y(), v.Z());
    }

    std::vector<HACD::Vec3<HACD::Real> > points;
    std::vector<HACD::Vec3<long> > triangles;

    btConvexHullShape *processCluster(HACD::HACD &hacd, int idx, btVector3 &ret_centroid);

public:
    ConvexDecomp(float margin_) : margin(margin_) { }
    void reset() { points.clear(); triangles.clear(); }
    void addPoint(const btVector3 &pt) {
        points.push_back(toHACDVec(pt));
    }
    void addTriangle(int v0, int v1, int v2) {
        triangles.push_back(HACD::Vec3<long>(v0, v1, v2));
    }
    //void addTriangle(const btVector3 &v0, const btVector3 &v1, const btVector3 &v2);
    boost::shared_ptr<btCompoundShape> run(std::vector<boost::shared_ptr<btCollisionShape> > &shapeStorage);
};


#endif // __CONVEXDECOMP_H__
