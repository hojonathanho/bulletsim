#include "collision_map_tools.h"
#include "utils/config.h"
#include <boost/foreach.hpp>
#include "simulation/basicobjects.h"
#include "simulation/simulation_fwd.h"
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
CollisionBoxes::Ptr collisionBoxesFromPointCloud(ColorCloudPtr cloud, float voxelSize) {
  std::vector<btVector3> sizes, centers;
  std::vector<osg::Vec4f> colors;
  btVector3 boxSize = btVector3(voxelSize, voxelSize, voxelSize)*METERS/2;
  BOOST_FOREACH(ColorPoint& pt, cloud->points) {
    centers.push_back(btVector3(pt.x, pt.y, pt.z)*METERS);
    sizes.push_back(boxSize);
    colors.push_back(osg::Vec4f(pt.r/255., pt.g/255., pt.b/255., 1));
  }
  return CollisionBoxes::Ptr(new CollisionBoxes(centers, sizes, colors));

}

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

inline btVector3 toBulletVector(Point& pt) {
	return btVector3(pt.x, pt.y, pt.z);
}

BulletObject::Ptr collisionMeshFromPointCloud(ColorCloudPtr colorCloud, float voxelSize) {

	  CloudPtr cloud = removeColor(colorCloud);

	  // Normal estimation*
	  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	  tree->setInputCloud (cloud);
	  n.setInputCloud (cloud);
	  n.setSearchMethod (tree);
	  n.setKSearch (20);
	  n.compute (*normals);
	  //* normals should not contain the point normals + surface curvatures

	  // Concatenate the XYZ and normal fields*
	  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	  //* cloud_with_normals = cloud + normals

	  // Create search tree*
	  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	  tree2->setInputCloud (cloud_with_normals);

	  // Initialize objects
	  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	  pcl::PolygonMesh triangles;

	  // Set the maximum distance between connected points (maximum edge length)
	  gp3.setSearchRadius (3*voxelSize);

	  // Set typical values for the parameters
	  gp3.setMu (2.5);
	  gp3.setMaximumNearestNeighbors (100);
	  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	  gp3.setMinimumAngle(M_PI/18); // 10 degrees
	  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	  gp3.setNormalConsistency(false);

	  // Get result
	  gp3.setInputCloud (cloud_with_normals);
	  gp3.setSearchMethod (tree2);
	  gp3.reconstruct (triangles);

	  // Additional vertex information
//	  std::vector<int> parts = gp3.getPartIDs();
//	  std::vector<int> states = gp3.getPointStates();

		btTriangleMesh* ptrimesh = new btTriangleMesh();
		for (size_t i = 0; i < triangles.polygons.size(); ++i) {
			vector<uint32_t> verts = triangles.polygons[i].vertices;
			ptrimesh->addTriangle(METERS*toBulletVector(cloud->points[verts[0]]), toBulletVector(cloud->points[verts[1]]), toBulletVector(cloud->points[verts[2]]));
		}
		btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape(ptrimesh, false);
		BulletObjectPtr meshobj(new BulletObject(0, shape, btTransform::getIdentity(),1));

		return meshobj;
}
