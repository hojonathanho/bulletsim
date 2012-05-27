#include "visibility.h"
#include "tracked_object.h"
#include "clouds/utils_pcl.h"
using namespace Eigen;

static const float DEPTH_OCCLUSION_DIST = .03;

VectorXf EverythingIsVisible::checkNodeVisibility(TrackedObject::Ptr obj) {
  return VectorXf::Ones(obj->m_nNodes);
}

VectorXf DepthImageVisibility::checkNodeVisibility(TrackedObject::Ptr obj) {
  MatrixXf ptsCam = m_transformer->toCamFromWorldMatrixXf(obj->getPoints());
  VectorXf ptDists = ptsCam.rowwise().norm();
  MatrixXi uvs = xyz2uv(ptsCam);
  VectorXf vis(ptsCam.rows(),true);

  for (int iPt=0; iPt<ptsCam.rows(); ++iPt) {
    int u = uvs(iPt,0);
    int v = uvs(iPt,1);
    if (u<m_depth.rows() && v<m_depth.cols() && u>0 && v>0) {
      vis[iPt] = isfinite(m_depth(u,v)) && (m_depth(u,v) + DEPTH_OCCLUSION_DIST > ptDists[iPt]);
    // see it if there's no non-rope pixel in front of it
    }
  }
  return vis;
}

void DepthImageVisibility::updateInput(const cv::Mat& in) {
	m_depth = toEigenMatrix(in);
}
