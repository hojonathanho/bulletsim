#include "visibility.h"
#include "tracked_object.h"
#include "clouds/utils_pcl.h"
#include "utils/conversions.h"
#include "simulation/config_bullet.h"
using namespace Eigen;

static const float DEPTH_OCCLUSION_DIST = .03;

VectorXf EverythingIsVisible::checkNodeVisibility(TrackedObject::Ptr obj) {
  return VectorXf::Ones(obj->m_nNodes);
}

VectorXf DepthImageVisibility::checkNodeVisibility(TrackedObject::Ptr obj) {
	  return VectorXf::Ones(obj->m_nNodes);
	MatrixXf ptsCam = toEigenMatrix(m_transformer->toCamFromWorldN(obj->getPoints()));
  VectorXf ptDists = ptsCam.rowwise().norm();
  MatrixXi uvs = xyz2uv(ptsCam);
  VectorXf vis(ptsCam.rows(),true);

  assert(m_depth.type() == CV_32FC1);
  float occ_dist = DEPTH_OCCLUSION_DIST*METERS;

  for (int iPt=0; iPt<ptsCam.rows(); ++iPt) {
    int u = uvs(iPt,0);
    int v = uvs(iPt,1);
    if (u<m_depth.rows && v<m_depth.cols && u>0 && v>0) {
      vis[iPt] = isfinite(m_depth.at<float>(u,v)) && (m_depth.at<float>(u,v) + occ_dist > ptDists[iPt]);
    // see it if there's no non-rope pixel in front of it
    }
  }
  return vis;
}

void DepthImageVisibility::updateInput(const cv::Mat& in) {
	m_depth = in;
}
