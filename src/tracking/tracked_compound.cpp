#include "tracked_object.h"
#include "config_tracking.h"
#include "utils/conversions.h"
#include "utils/cvmat.h"
#include "simulation/bullet_io.h"
#include "utils/utils_vector.h"

using namespace std;
using namespace Eigen;

TrackedArticulated::TrackedArticulated(CapsuleRope::Ptr sim) : TrackedObject(sim, "rope") {
  m_nNodes = sim->children.size();
  m_masses.resize(m_nNodes);
  for (int i=0; i < m_nNodes; i++) {
    m_masses(i) = 1/getSim()->children[i]->rigidBody->getInvMass();
  }
}

std::vector<btVector3> TrackedArticulated::getPoints() {
	std::vector<btVector3> out(m_nNodes);
	for (int i=0; i < m_nNodes; ++i) {
		out[i] = getSim()->children[i]->rigidBody->getCenterOfMassPosition();
	}
	return out;
}

const VectorXf TrackedArticulated::getPriorDist() {
	VectorXf prior_dist(6);
	prior_dist << TrackingConfig::pointPriorDist*METERS, TrackingConfig::pointPriorDist*METERS, TrackingConfig::pointPriorDist*METERS, 0.3, 0.15, 0.15;
	cout << "fixme: TrackedArticulated::getPriorDist()" << endl;
	return prior_dist;
}

void TrackedArticulated::applyEvidence(const Eigen::MatrixXf& corr, const MatrixXf& obsPts) {
  vector<btVector3> estPos(m_nNodes), estVel(m_nNodes);
  for (int i=0; i < m_nNodes; ++i)  {
    estPos[i] = getSim()->children[i]->rigidBody->getCenterOfMassPosition();
    estVel[i] = getSim()->children[i]->rigidBody->getLinearVelocity();
  }
  vector<btVector3> impulses = calcImpulsesDamped(estPos, estVel, toBulletVectors(obsPts.leftCols(3)), corr, toVec(m_masses), TrackingConfig::kp_rope, TrackingConfig::kd_rope);

  for (int i=0; i<m_nNodes; ++i) getSim()->children[i]->rigidBody->applyCentralImpulse(impulses[i]);
}

void TrackedArticulated::initColors() {
	for (int i=0; i < m_nNodes; ++i) {
		Vector3f bgr = toEigenMatrixImage(getSim()->children[i]->getTexture()).colwise().mean();
		m_colors.row(i) = bgr.transpose();
	}
}

cv::Mat TrackedArticulated::makeTexture(ColorCloudPtr cloud) {
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
}
