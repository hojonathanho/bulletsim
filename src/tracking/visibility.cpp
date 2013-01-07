#include "visibility.h"
#include "tracked_object.h"
#include "clouds/utils_pcl.h"
#include "utils/conversions.h"
#include "simulation/config_bullet.h"
#include "utils/my_assert.h"
#include "simulation/bullet_io.h"
#include "simulation/util.h"

using namespace Eigen;

static const float DEPTH_OCCLUSION_DIST = .03;
static const float RAY_SHORTEN_DIST = .0001;
static const float MIN_DEPTH = .4;

// 1's if the point is a no observation point
// a no observation point lies above the table and behind a real object
VectorXf ObservationVisibility::checkObservationVisibility(ColorCloud::Ptr cloud) {
  MatrixXf eigen_cloud = toEigenMatrix(cloud);
	MatrixXf ptsCam = m_transformer->toCamFromWorldMatrixXf(eigen_cloud);
  VectorXf ptDists = ptsCam.rowwise().norm();
  MatrixXi uvs = xyz2uv(ptsCam);
  VectorXf vis(ptsCam.rows(),true);

  assert(m_depth.type() == CV_32FC1);

  for (int iPt=0; iPt<ptsCam.rows(); ++iPt) {
    int u = uvs(iPt,0);
    int v = uvs(iPt,1);
    if (cloud->at(iPt).z < 0)
    	vis[iPt] = false;
    if (u<m_depth.rows && v<m_depth.cols && u>0 && v>0) {
      if (!isfinite(m_depth.at<float>(u,v)) || (m_depth.at<float>(u,v) > ptDists[iPt]))
      	vis[iPt] = false;
    }
  }
  return vis;
}

VectorXf EverythingIsVisible::checkNodeVisibility(TrackedObject::Ptr obj) {
  return VectorXf::Ones(obj->m_nNodes);
}


VectorXf DepthImageVisibility::checkNodeVisibility(TrackedObject::Ptr obj) {
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
      vis[iPt] = !isfinite(m_depth.at<float>(u,v)) || (m_depth.at<float>(u,v) + occ_dist > ptDists[iPt]);
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
			if (sim->checkIntersection(points[i]-RAY_SHORTEN_DIST*METERS*view, cam_pos)) {
				vis(i) = 0;
			} else {
				vis(i) = 1;
//				vis(i) = abs(view.dot(normals[i]));
//				assert(vis(i)>=0);
//				assert(vis(i)<=1);
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
		vector<btVector3> inter_points_partial = sim->getIntersectionPoints(points[i]+RAY_SHORTEN_DIST*METERS*point_to_cam, cam_pos);
		for (int j=0; j<inter_points_partial.size(); j++)
			inter_points.push_back(inter_points_partial[j]);
	}
	return inter_points;
}


BulletRaycastVisibility::BulletRaycastVisibility(btDynamicsWorld* world, CoordinateTransformer* transformer)
	: m_world(world), m_transformer(transformer) {
}

//#define PLOT_RAYCAST

#ifdef PLOT_RAYCAST
#include "plotting_tracking.h"
#endif

VectorXf BulletRaycastVisibility::checkNodeVisibility(TrackedObject::Ptr obj) {
	vector<btVector3> nodes = obj->getPoints();
	btVector3 cameraPos = m_transformer->worldFromCamUnscaled.getOrigin()*METERS;
	VectorXf vis(nodes.size());

	float ray_shorten_dist = RAY_SHORTEN_DIST*METERS;
	float min_depth = MIN_DEPTH*METERS;

#ifdef PLOT_RAYCAST
    static PlotLines::Ptr rayTestLines;
    static PlotPoints::Ptr rayHitPoints;
    if (!rayTestLines) {
      rayTestLines.reset(new PlotLines(4));
      rayHitPoints.reset(new PlotPoints(30));
      util::getGlobalEnv()->add(rayTestLines);
      rayTestLines->setColor(1,1,1,1);
      printf("adding ray test lines to env\n");
    }
    vector<btVector3> linePoints;
    vector<btVector3> hitPoints;
#endif

	for (int i=0; i < nodes.size(); ++i) {
	  btVector3 rayDir = (nodes[i] - cameraPos).normalized();
		btVector3 rayEnd = nodes[i] - ray_shorten_dist * rayDir;
		btVector3 rayStart = cameraPos + min_depth * rayDir;
		btCollisionWorld::ClosestRayResultCallback rayCallback(rayStart, rayEnd);
		m_world->rayTest(rayStart, rayEnd, rayCallback);
		vis[i] = !rayCallback.hasHit();

#ifdef PLOT_RAYCAST
		if (rayCallback.hasHit()) hitPoints.push_back(rayCallback.m_hitPointWorld);
		linePoints.push_back(rayStart);
		linePoints.push_back(rayEnd);
#endif
	}
#ifdef PLOT_RAYCAST
	rayTestLines->setPoints(linePoints);
#endif
	return vis;
}
