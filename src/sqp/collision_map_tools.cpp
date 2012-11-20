#include "collision_map_tools.h"
#include "utils/config.h"
#include <boost/foreach.hpp>
CollisionBoxes::Ptr collisionBoxesFromPointCloud(ColorCloudPtr cloud, float voxelSize) {
  std::vector<btVector3> sizes, centers;
  std::vector<osg::Vec4f> colors;
  btVector3 boxSize = btVector3(voxelSize, voxelSize, 10*voxelSize)*METERS/2;
  BOOST_FOREACH(ColorPoint& pt, cloud->points) {
    centers.push_back(btVector3(pt.x, pt.y, pt.z-4*voxelSize)*METERS);
    sizes.push_back(boxSize);
    colors.push_back(osg::Vec4f(pt.r/255., pt.g/255., pt.b/255., 1));
  }
  return CollisionBoxes::Ptr(new CollisionBoxes(centers, sizes, colors));

}
