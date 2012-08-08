#include "tracked_compound.h"
#include "config_tracking.h"
#include "utils/conversions.h"
#include "utils/cvmat.h"
#include "simulation/bullet_io.h"
#include "utils/utils_vector.h"
#include "tracking/surface_sampling.h"
#include <boost/foreach.hpp>

using namespace std;
using namespace Eigen;

TrackedCompound::TrackedCompound(GenericCompoundObject::Ptr sim, btDynamicsWorld* world) : TrackedObject(sim, "compound") {
  m_nBodies = sim->children.size();
  VectorXf bodyMasses(m_nBodies);
  for (int i=0; i < m_nBodies; i++) {
	sim->children[i];
	sim->children[i]->rigidBody;
    bodyMasses(i) = sim->children[i]->rigidBody->getInvMass();
  }

  getSampledDescription(sim, world, .015*METERS, m_node2body, m_relativePos);
  m_nNodes = m_node2body.size();
  for (int i=0; i < m_nNodes; ++i) {
	  m_body2nodes[m_node2body[i]].push_back(i);
  }

  m_masses.resize(m_nNodes);
  for (int i=0; i < m_nBodies; ++i) {
	  vector<int>& nodes = m_body2nodes[i];
	  BOOST_FOREACH(int node, nodes) m_masses(node) = bodyMasses(i) / nodes.size();
  }

}

std::vector<btVector3> TrackedCompound::getPoints() {
	vector<btTransform> transforms;
	transforms.reserve(m_nBodies);
	BOOST_FOREACH(BulletObject::Ptr& child, getSim()->children) transforms.push_back(child->rigidBody->getCenterOfMassTransform());

	vector<btVector3> out(m_nNodes);
	for (int i=0; i < m_nNodes; ++i) {
		int bodyInd = m_node2body[i];
		out[i] = transforms[bodyInd] * m_relativePos[i];
	}
	return out;
}

const VectorXf TrackedCompound::getPriorDist() {
	VectorXf prior_dist(6);
	prior_dist << TrackingConfig::pointPriorDist*METERS, TrackingConfig::pointPriorDist*METERS, TrackingConfig::pointPriorDist*METERS, 0.3, 0.15, 0.15;
	cout << "fixme: TrackedCompound::getPriorDist()" << endl;
	return prior_dist;
}

void TrackedCompound::applyEvidence(const Eigen::MatrixXf& corr, const MatrixXf& obsPts) {
	vector<btTransform> transforms;
	transforms.reserve(m_nBodies);
	vector<btRigidBody*> bodies;
	bodies.reserve(m_nBodies);
	BOOST_FOREACH(BulletObject::Ptr& child, getSim()->children) {
		transforms.push_back(child->rigidBody->getCenterOfMassTransform());
		bodies.push_back(child->rigidBody.get());
	}
	vector<btVector3> pos(m_nNodes);
	vector<btVector3> vel(m_nNodes);
	for (int i=0; i < m_nNodes; ++i) {
		int bodyInd = m_node2body[i];
		pos[i] = transforms[bodyInd] * m_relativePos[i];
		vel[i] = bodies[bodyInd]->getVelocityInLocalPoint(m_relativePos[i]);
	}
  vector<btVector3> impulses = calcImpulsesDamped(pos, vel, toBulletVectors(obsPts.leftCols(3)), corr, toVec(m_masses), TrackingConfig::kp_rope, TrackingConfig::kd_rope);

	for (int i=0; i<m_nNodes; ++i) {
	  getSim()->children[m_node2body[i]]->rigidBody->applyImpulse(impulses[i], m_relativePos[i]);
	}

}

void TrackedCompound::initColors() {
	TrackedObject::initColors();
	return;
	for (int i=0; i < m_nBodies; ++i) {
		Vector3f bgr = toEigenMatrixImage(getSim()->children[i]->getTexture()).colwise().mean();
		m_colors.row(i) = bgr.transpose();
	}
}

cv::Mat TrackedCompound::makeTexture(ColorCloudPtr cloud) {
#if 0
	vector<btVector3> nodes = getPoints();
	int x_res = 3;
	int ang_res = 1;
	cv::Mat image(ang_res, nodes.size()*x_res, CV_8UC3);
	vector<btMatrix3x3> rotations = getSim()->getRotations();
	vector<float> half_heights = getSim()->getHalfHeights();
	for (int j=0; j<nodes.size(); j++) {
		pcl::KdTreeFLANN<ColorPoint> kdtree;
		kdtree.setInputCloud(cloud);
		ColorPoint searchPoint;
		searchPoint.x = nodes[j].x();
		searchPoint.y = nodes[j].y();
		searchPoint.z = nodes[j].z();
		// Neighbors within radius search
//		float radius = ((float) TrackingConfig::fixeds)/10.0; //(fixeds in cm)
		float radius = ((float) 3)/10.0; //(fixeds in cm)
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		Eigen::Matrix3f node_rot = toEigenMatrix(rotations[j]);
		float node_half_height = half_heights[j];
		vector<vector<float> > B_bins(ang_res*x_res), G_bins(ang_res*x_res), R_bins(ang_res*x_res);
		if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++) {
				Eigen::Vector3f alignedPoint = node_rot * (toEigenVector(cloud->points[pointIdxRadiusSearch[i]]) - toEigenVector(searchPoint));
				int xId = (int) floor( ((float) x_res) * (1.0 + alignedPoint(0)/node_half_height) * 0.5 );
				float angle = atan2(alignedPoint(2), alignedPoint(1))*180.0/M_PI;
				if (angle >= 90) angle-=90;
				else angle+=270;
				angle = 360-angle;
				//if (angle<0) angle+=360.0;
				int angId = (int) floor( ((float) ang_res) * angle/360.0 );
				assert(angId >= 0 && angId < ang_res);
				if (xId >= 0 && xId < x_res) {
					B_bins[xId*ang_res+angId].push_back(cloud->points[pointIdxRadiusSearch[i]].b);
					G_bins[xId*ang_res+angId].push_back(cloud->points[pointIdxRadiusSearch[i]].g);
					R_bins[xId*ang_res+angId].push_back(cloud->points[pointIdxRadiusSearch[i]].r);
				}
//					if (xId >= 0 && xId < x_res/2 && j%2==0) {
//						debugCloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
//					}
			}
		}
//		for (int xId=0; xId<x_res; xId++) {
//			for (int angId=0; angId<ang_res; angId++) {
//				image.at<cv::Vec3b>(angId,j*x_res+xId)[0] = mean(append(B_bins, xId*ang_res, (xId+1)*ang_res));
//				image.at<cv::Vec3b>(angId,j*x_res+xId)[1] = mean(append(G_bins, xId*ang_res, (xId+1)*ang_res));
//				image.at<cv::Vec3b>(angId,j*x_res+xId)[2] = mean(append(R_bins, xId*ang_res, (xId+1)*ang_res));
//			}
//		}
		for (int xId=0; xId<x_res; xId++) {
			for (int angId=0; angId<ang_res; angId++) {
				image.at<cv::Vec3b>(angId,j*x_res+xId)[0] = mean(B_bins[xId*ang_res+angId]);
				image.at<cv::Vec3b>(angId,j*x_res+xId)[1] = mean(G_bins[xId*ang_res+angId]);
				image.at<cv::Vec3b>(angId,j*x_res+xId)[2] = mean(R_bins[xId*ang_res+angId]);
			}
		}
	}
	return image;
#endif
}
