/*
 * tracked_sponge.cpp
 *
 *  Created on: Sep 13, 2012
 *      Author: alex
 */

#include "tracked_object.h"
#include <boost/foreach.hpp>
#include <utility>
#include "config_tracking.h"
#include "simulation/config_bullet.h"
#include "utils/conversions.h"
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <boost/format.hpp>
#include "simulation/bullet_io.h"
#include <pcl/common/transforms.h>
#include "clouds/utils_pcl.h"
#include "feature_extractor.h"
#include <algorithm>
#include "utils/utils_vector.h"
#include "utils/cvmat.h"
#include "simulation/softBodyHelpers.h"
#include "tracking/utils_tracking.h"
#include "clouds/plane_finding.h"
#include "utils_tracking.h"

//DEBUG
#include "simulation/util.h"
#include <pcl/io/pcd_io.h>

//Do you need the includes above?

#include "clouds/cloud_ops.h"
#include "simulation/tetgen_helpers.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

using namespace std;
using namespace Eigen;

TrackedSponge::TrackedSponge(BulletSoftObject::Ptr sim) : TrackedObject(sim, "sponge") {
	for (int i=0; i<getSim()->softBody->m_nodes.size(); i++) {
		m_node2vert.push_back(i);
		m_vert2nodes.push_back(vector<int>(1,i));
	}
  m_nNodes = m_node2vert.size();

	const btSoftBody::tNodeArray& verts = getSim()->softBody->m_nodes;
  m_masses.resize(m_nNodes);
  for (int i=0; i < m_nNodes; ++i) {
    m_masses(i) = 1/verts[m_node2vert[i]].m_im;
  }
}

void TrackedSponge::applyEvidence(const Eigen::MatrixXf& corr, const Eigen::MatrixXf& obsPts) {
  vector<btVector3> estPos(m_nNodes);
  vector<btVector3> estVel(m_nNodes);

  btAlignedObjectArray<btSoftBody::Node>& verts = getSim()->softBody->m_nodes;
  for (int iNode=0; iNode < m_nNodes; ++iNode)  {
    estPos[iNode] = verts[m_node2vert[iNode]].m_x;
    estVel[iNode] = verts[m_node2vert[iNode]].m_v;
  }

  vector<btVector3> nodeImpulses = calcImpulsesDamped(estPos, estVel, toBulletVectors(obsPts), corr, toVec(m_masses), TrackingConfig::kp_cloth, TrackingConfig::kd_cloth);

  int nVerts = verts.size();

  for (int iVert=0; iVert < nVerts; iVert++) {
    btVector3 impulse(0,0,0);
    BOOST_FOREACH(int iNode, m_vert2nodes[iVert]) impulse += nodeImpulses[iNode];
    impulse/= m_vert2nodes[iVert].size();
    getSim()->softBody->addForce(impulse, iVert);

    assert(isfinite(impulse.x()) && isfinite(impulse.y()) && isfinite(impulse.z()));
  }
}

vector<btVector3> TrackedSponge::getPoints() {
	vector<btVector3> out(m_nNodes);
	btAlignedObjectArray<btSoftBody::Node>& verts = getSim()->softBody->m_nodes;
	for (int iNode=0; iNode < m_nNodes; ++iNode)
		out[iNode] = verts[m_node2vert[iNode]].m_x;
	return out;
}

const Eigen::VectorXf TrackedSponge::getPriorDist() {
	Eigen::MatrixXf prior_dist(1,FeatureExtractor::m_allDim);
	prior_dist << TrackingConfig::pointPriorDist*METERS, TrackingConfig::pointPriorDist*METERS, TrackingConfig::pointPriorDist*METERS,  //FT_XYZ
			0.2, 0.2, 0.2, 	//FT_BGR
			TrackingConfig::colorLPriorDist, TrackingConfig::colorABPriorDist, TrackingConfig::colorABPriorDist,	//FT_LAB
			1.0, 1.0, 1.0,  //FT_NORMAL
			1.0,  //FT_LABEL
			MatrixXf::Ones(1, FE::FT_SIZES[FE::FT_SURF])*0.4,  //FT_SURF
			MatrixXf::Ones(1, FE::FT_SIZES[FE::FT_PCASURF])*0.4,  //FT_PCASURF
			0.5;  //FT_GRADNORMAL
	return FeatureExtractor::all2ActiveFeatures(prior_dist).transpose();
}

// Assumes top_corners are in a plane parallel to the xy-plane
// The bottom corners are the top_corners shifted by thickness in the negative z direction
// good max_tet_vol: 0.0008*METERS*METERS*METERS
BulletSoftObject::Ptr makeSponge(const vector<btVector3>& top_corners, float thickness, float max_tet_vol, float mass) {
	assert(thickness > 0);
  btSoftBodyWorldInfo unusedWorldInfo;
  btVector3 polygon_translation(0,0,thickness);
  vector<btVector3> bottom_corners;
  BOOST_FOREACH(const btVector3& top_corner, top_corners) bottom_corners.push_back(top_corner - polygon_translation);
	btSoftBody* psb=CreatePrism(unusedWorldInfo, bottom_corners, polygon_translation, 1.414, max_tet_vol, false,true,true);

  btSoftBody::Material* pm=psb->appendMaterial();
  pm->m_kLST = 0.4;

  //psb->generateBendingConstraints(2,pm);

	psb->setVolumeMass(mass);

  psb->generateClusters(16);
	psb->getCollisionShape()->setMargin(0.001*METERS);

  psb->m_cfg.collisions	=	0;
  //psb->m_cfg.collisions += btSoftBody::fCollision::SDF_RS; ///SDF based rigid vs soft
  psb->m_cfg.collisions += btSoftBody::fCollision::CL_RS; ///Cluster vs convex rigid vs soft
  //psb->m_cfg.collisions += btSoftBody::fCollision::VF_SS;	///Vertex vs face soft vs soft handling
  psb->m_cfg.collisions += btSoftBody::fCollision::CL_SS; ///Cluster vs cluster soft vs soft handling
  //psb->m_cfg.collisions	+= btSoftBody::fCollision::CL_SELF; ///Cluster soft body self collision

  psb->m_cfg.kDF = 1.0; // Dynamic friction coefficient

	psb->m_cfg.piterations=1;

  BulletSoftObject::Ptr bso = BulletSoftObject::Ptr(new BulletSoftObject(psb));

	return bso;
}


// Returns the approximate polygon of the concave hull of the cloud
// The points are being projected to the xy plane
vector<btVector3> polyCorners(ColorCloudPtr cloud) {
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	filterPlane(cloud, 0.01*METERS, coefficients);
	ColorCloudPtr cloud_projected = projectOntoPlane(cloud, coefficients);
	vector<pcl::Vertices> polygons;
	ColorCloudPtr cloud_hull = findConcaveHull(cloud_projected, 0.03*METERS, polygons);

	vector<btVector3> pts = toBulletVectors(cloud_hull);

	vector<float> pts_z;
	for (int i=0; i<pts.size(); i++)
		pts_z.push_back(pts[i].z());
	float median_z = median(pts_z);

	vector<cv::Point2f> pts_2d;
	for (int i=0; i<pts.size(); i++)
		pts_2d.push_back(cv::Point2f(pts[i].x(), pts[i].y()));

	vector<cv::Point2f> pts_approx_2d;
	cv::approxPolyDP(cv::Mat(pts_2d), pts_approx_2d, 0.01*METERS, true);

	vector<btVector3> pts_approx;
	for (int i=0; i<pts_approx_2d.size(); i++)
		pts_approx.push_back(btVector3(pts_approx_2d[i].x, pts_approx_2d[i].y, median_z));

	return pts_approx;
}
