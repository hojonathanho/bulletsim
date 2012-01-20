#include "convexdecomp.h"

void ConvexDecomp::addTriangle(const btVector3 &v0, const btVector3 &v1, const btVector3 &v2) {
    points.push_back(toHACDVec(v0));
    points.push_back(toHACDVec(v1));
    points.push_back(toHACDVec(v2));
    triangles.push_back(HACD::Vec3<long>(points.size() - 3, points.size() - 2, points.size() - 1));
}

boost::shared_ptr<btCompoundShape> ConvexDecomp::run(std::vector<boost::shared_ptr<btCollisionShape> > &shapeStorage) {
    HACD::HACD hacd;
    hacd.SetPoints(&points[0]);
    hacd.SetNPoints(points.size());
    hacd.SetTriangles(&triangles[0]);
    hacd.SetNTriangles(triangles.size());
    hacd.SetCompacityWeight(0.1);
    hacd.SetVolumeWeight(0.0);

    // HACD parameters
    // Recommended parameters: 2 100 0 0 0 0
    size_t nClusters = 2;
    double concavity = 100;
    bool invert = false;
    bool addExtraDistPoints = false;
    bool addNeighboursDistPoints = false;
    bool addFacesPoints = false;       

    hacd.SetNClusters(nClusters);                     // minimum number of clusters
    hacd.SetNVerticesPerCH(100);                      // max of 100 vertices per convex-hull
    hacd.SetConcavity(concavity);                     // maximum concavity
    hacd.SetAddExtraDistPoints(addExtraDistPoints);   
    hacd.SetAddNeighboursDistPoints(addNeighboursDistPoints);   
    hacd.SetAddFacesPoints(addFacesPoints); 

    hacd.Compute();
    nClusters = hacd.GetNClusters();	

    boost::shared_ptr<btCompoundShape> compound(new btCompoundShape());
    for (int c = 0; c < nClusters; ++c) {
        btVector3 centroid;
        boost::shared_ptr<btConvexHullShape> shape(processCluster(hacd, c, centroid));
        shapeStorage.push_back(shape);
        compound->addChildShape(btTransform(btQuaternion(0, 0, 0, 1), centroid), shape.get());
    }

    return compound;
}

btConvexHullShape *ConvexDecomp::processCluster(HACD::HACD &hacd, int c, btVector3 &ret_centroid) {
    //generate convex result
    size_t nPoints = hacd.GetNPointsCH(c);
    size_t nTriangles = hacd.GetNTrianglesCH(c);

    HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
    HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
    hacd.GetCH(c, pointsCH, trianglesCH);

    // points
    ret_centroid.setZero();
    for(size_t v = 0; v < nPoints; v++)
        ret_centroid += toBtVector(pointsCH[v]);
    ret_centroid *= 1./nPoints;

    btAlignedObjectArray<btVector3> shiftedVertices;
    shiftedVertices.reserve(nPoints);
    for(size_t v = 0; v < nPoints; v++)
        shiftedVertices.push_back(toBtVector(pointsCH[v]) - ret_centroid);

    delete [] pointsCH;
    delete [] trianglesCH;

    btConvexHullShape *shape = new btConvexHullShape(&shiftedVertices[0].getX(), nPoints);
    shape->setMargin(margin);

    return shape;
}
