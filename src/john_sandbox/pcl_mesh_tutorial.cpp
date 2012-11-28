#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
#include "simulation/simplescene.h"

int main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  ColorCloudPtr bigCloud = readPCD("/home/joschu/Data/scp/three_objs.pcd");
  ColorCloudPtr dsCloud = downsampleCloud(bigCloud,.01);
  CloudPtr cloud = removeColor(dsCloud);

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
  gp3.setSearchRadius (0.025);

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
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

	btTriangleMesh* ptrimesh = new btTriangleMesh();
	for (size_t i = 0; i < triangles.polygons.size(); ++i) {
		vector<uint32_t> verts = triangles.polygons[i].vertices;
		ptrimesh->addTriangle(toBtVector(cloud->points[verts[0]]), toBtVector(cloud->points[verts[1]]), toBtVector(cloud->points[verts[2]]));
	}
	btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape(ptrimesh, false);
	BulletObjectPtr meshobj(new BulletObject(0, shape, btTransform::getIdentity(),1));
	Scene scene;
	scene.env->add(meshobj);
	scene.startViewer();
	scene.idle(true);
}
