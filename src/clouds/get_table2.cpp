#include "clouds/cloud_ops.h"
#include "clouds/utils_pcl.h"
#include "clouds/plane_finding.h"
#include "clouds/get_table2.h"
#include "clouds/geom.h"
#include <boost/foreach.hpp>
using namespace std;
using namespace Eigen;
using namespace pcl;


vector<Vector3f> getTableCornersRansac(ColorCloudPtr cloud) {
  cloud = downsampleCloud(cloud, .01);
  ColorCloudPtr fovCenter = filterX(cloud,-.25, .25); // center of field of view
  vector<float> coeffs = getPlaneCoeffsRansac(fovCenter);

  vector<int> inliers = getPlaneInliers(cloud, coeffs);
  
  ColorCloudPtr inlierCloud = extractInds(cloud, inliers);
  ColorCloudPtr bigClu = getBiggestCluster(inlierCloud,.025);
  
  vector<Vector3f> tablePts, corners, tableVerts;
  Vector4f abcd(coeffs.data());
  BOOST_FOREACH(PointXYZRGBA& pt, bigClu->points) tablePts.push_back(Vector3f(pt.x,pt.y,pt.z));
  minEncRect(tablePts,abcd,corners);

  return corners;
  
  
}