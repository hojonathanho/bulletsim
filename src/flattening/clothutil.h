#ifndef __CLOTHMANIP_H__
#define __CLOTHMANIP_H__

#include "simulation/environment.h"
#include "simulation/basicobjects.h"

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

class Scene;
class btSoftBody;

struct ClothSpec {
    btSoftBody *psb;
    int resx, resy;
    btScalar lenx, leny;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::KdTree<pcl::PointXYZ>::Ptr kdtree;

    ClothSpec() { }
    ClothSpec(btSoftBody *psb_, int resx_, int resy_, btScalar lenx_, btScalar leny_);

    void updateAccel(); // update acceleration structures (kdtree, etc)
};

#if 0
#define CLOTHIDX(cs,x,y) ((y)*(cs).resx+(x))
#define CLOTHIDX_X(cs,i) ((i) % (cs).resx)
#define CLOTHIDX_Y(cs,i) ((i) / (cs).resx)

#define CLOTHCOORDS_OOB(cs,x,y) \
    ((x) < 0 || (x) >= (cs).resx \
     || (y) < 0 || (y) >= (cs).resy)

#define CLOTHIDX_ON_TOP_EDGE(cs, i) \
    ((i) < CLOTHIDX((cs), 0, 1))
#define CLOTHIDX_ON_BOTTOM_EDGE(cs, i) \
     ((i) >= CLOTHIDX((cs), 0, (cs).resy-1) \
      && (i) <= CLOTHIDX((cs), (cs).resx-1, (cs).resy-1))
#define CLOTHIDX_ON_LEFT_EDGE(cs, i) \
     (CLOTHIDX_X((cs), (i)) == 0)
#define CLOTHIDX_ON_RIGHT_EDGE(cs, i) \
     (CLOTHIDX_X((cs), (i)) == (cs).resx-1)
#endif

// utility functions for manipulating cloth
void liftClothEdges(Scene &scene, ClothSpec &cs, bool disableDrawing=true);
void liftClothMiddle(Scene &scene, ClothSpec &cs, bool disableDrawing=true);
void randomizeCloth(Scene &scene, btSoftBody *psb, int resx, int resy, int numSteps=20);

// calculates the number of faces that the ray crosses
int softBody_facesCrossed(const btSoftBody* psb, const btVector3& rayFrom, const btVector3& rayTo);

#endif // __CLOTHMANIP_H__
