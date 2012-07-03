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
	//  return VectorXf::Ones(obj->m_nNodes);
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

VectorXf OSGVisibility::checkNodeVisibility(TrackedObject::Ptr obj) {
	VectorXf vis(obj->m_nNodes);
	if (obj->m_type == "towel") {
		BulletSoftObject* sim = dynamic_cast<BulletSoftObject*>(obj->getSim());
		btVector3 cam_pos = m_transformer->toWorldFromCam(btVector3(0,0,0));
		vector<btVector3> points = obj->getPoints();
		vector<btVector3> normals = dynamic_cast<TrackedTowel*>(obj.get())->getNormals();
		for (int i=0; i<points.size(); i++) {
			btVector3 view = (points[i]-cam_pos).normalized();
			if (sim->checkIntersection(points[i]-0.01*view, cam_pos)) {
				vis(i) = 0;
			} else {
				vis(i) = abs(view.dot(normals[i]));
				assert(vis(i)>=0);
				assert(vis(i)<=1);
			}
		}
	} else {
		vis = VectorXf::Ones(obj->m_nNodes);
	}
	return vis;
}

vector<btVector3> OSGVisibility::getIntersectionPoints(TrackedObject::Ptr obj) {
	BulletSoftObject* sim = dynamic_cast<BulletSoftObject*>(obj->getSim());
	vector<btVector3> points = obj->getPoints();

	btVector3 cam_pos = m_transformer->toWorldFromCam(btVector3(0,0,0));

	vector<btVector3> inter_points;
	for (int i=0; i<points.size(); i++) {
		btVector3 point_to_cam = (cam_pos-points[i]).normalized();
		vector<btVector3> inter_points_partial = sim->getIntersectionPoints(points[i]+0.01*point_to_cam, cam_pos);
		for (int j=0; j<inter_points_partial.size(); j++)
			inter_points.push_back(inter_points_partial[j]);
	}
	return inter_points;
}
