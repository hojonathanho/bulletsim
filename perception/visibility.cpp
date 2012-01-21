#include "rope.h"
#include "utils_perception.h"
#include "clouds/utils_pcl.h"
using namespace Eigen;
using namespace pcl;


// todo: we should do a raycast
std::vector<float> calcVisibility(const Eigen::MatrixXf& pts, const Eigen::MatrixXf& depth, const cv::Mat& ropeMask) {
  VectorXf ptDists = pts.rowwise().norm();
  MatrixXi uvs = xyz2uv(pts);
  vector<bool> vis(pts.size());

  for (int iPt=0; iPt<pts.rows(); iPt++) {
    int u = pts(iPt,0);
    int v = pts(iPt,1);
    vis[iPt] = ropeMask.at<uint8_t>(u,v) || isfinite(depth(u,v)) && (depth(u,v) + .04 > ptDists[iPt]);
    // see it if there's no non-rope pixel in front of it
  }
}
