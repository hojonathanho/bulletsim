#include "rope.h"
#include "utils_perception.h"
#include "clouds/utils_pcl.h"
#include "vector_io.h"
using namespace Eigen;
using namespace pcl;


// todo: we should do a raycast
std::vector<float> calcVisibility(const Eigen::MatrixXf& pts, const Eigen::MatrixXf& depth, const cv::Mat& ropeMask) {
  VectorXf ptDists = pts.rowwise().norm();
  MatrixXi uvs = xyz2uv(pts);
  vector<float> vis(pts.rows(),true);

  for (int iPt=0; iPt<pts.rows(); iPt++) {
    int u = uvs(iPt,0);
    int v = uvs(iPt,1);
    if (u<depth.rows() && v<depth.cols() && u>0 && v>0) {
      vis[iPt] = ropeMask.at<uint8_t>(u,v) || isfinite(depth(u,v)) && (depth(u,v) + .04 > ptDists[iPt]);
    // see it if there's no non-rope pixel in front of it
    }
  }
  cout << vis << endl;
  return vis;
}

void colorByVisibility(CapsuleRope::Ptr rope, const vector<float>& pVis) {
  assert(rope->children.size() == pVis.size());
  for (int i=0; i<pVis.size(); i++) {
    if (pVis[i] < .5) rope->children[i]->setColor(0,0,0,1);
    else rope->children[i]->setColor(1,1,1,1);
  }
}
