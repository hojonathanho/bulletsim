#include "tracked_object.h"
#include "config_tracking.h"
#include "utils/conversions.h"
#include "simulation/bullet_io.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace Eigen;

TrackedBox::TrackedBox(BoxObject::Ptr sim) : TrackedObject(sim, "box") {
	BoxObject* box_object = dynamic_cast<BoxObject*>(m_sim.get());
	btCollisionShape* bt_collision_shape = dynamic_cast<btCollisionShape*>(box_object->collisionShape.get());
	btBoxShape* bt_box_shape = dynamic_cast<btBoxShape*>(bt_collision_shape);
	btVector3 half_extents = bt_box_shape->getHalfExtentsWithMargin();
	m_nEdgeNodesX = half_extents.x()/0.2;
	m_nEdgeNodesY = half_extents.y()/0.2;
	m_nEdgeNodesZ = half_extents.z()/0.2;
	m_nNodes = m_nEdgeNodesX*m_nEdgeNodesY*m_nEdgeNodesZ - (m_nEdgeNodesX-2)*(m_nEdgeNodesY-2)*(m_nEdgeNodesZ-2);
	//m_nNodes = pow(m_nEdgeNodes, 3) - pow(m_nEdgeNodes-2, 3);
	m_masses.resize(m_nNodes);
	float invMass = getSim()->rigidBody->getInvMass();
	for (int i=0; i < m_nNodes; i++) {
		m_masses(i) = (1/invMass)/m_nNodes;
	}
	m_nFeatures = 3;
}

std::vector<btVector3> TrackedBox::getPoints() {
	BoxObject* box_object = dynamic_cast<BoxObject*>(m_sim.get());
	btCollisionShape* bt_collision_shape = dynamic_cast<btCollisionShape*>(box_object->collisionShape.get());
	btBoxShape* bt_box_shape = dynamic_cast<btBoxShape*>(bt_collision_shape);
	btVector3 half_extents = bt_box_shape->getHalfExtentsWithMargin();
	//std::vector<btVector3> out(m_nNodes);
	std::vector<btVector3> out;
	float midNEdgeNodesX = 0.5*((float) m_nEdgeNodesX - 1.0);
	float midNEdgeNodesY = 0.5*((float) m_nEdgeNodesY - 1.0);
	float midNEdgeNodesZ = 0.5*((float) m_nEdgeNodesZ - 1.0);

	btTransform cm = getSim()->rigidBody->getCenterOfMassTransform();
	for (int i=0; i < m_nEdgeNodesX; i++) {
		for (int j=0; j < m_nEdgeNodesY; j++) {
			for (int k=0; k < m_nEdgeNodesZ; k++) {
				if (i!=0 && i!=(m_nEdgeNodesX-1) && j!=0 && j!=(m_nEdgeNodesY-1) && k!=0 && k!=(m_nEdgeNodesZ-1)) continue;
				float i_offset = ((float) i)/midNEdgeNodesX - 1.0;
                float j_offset = ((float) j)/midNEdgeNodesY - 1.0;
                float k_offset = ((float) k)/midNEdgeNodesZ - 1.0;
				//out[(int)pow(m_nEdgeNodes, 2.0)*i+m_nEdgeNodes*j+k] = cm.getOrigin() + cm.getBasis() * (half_extents * btVector3(i_offset, j_offset, k_offset));
                out.push_back(cm.getOrigin() + cm.getBasis() * (half_extents * btVector3(i_offset, j_offset, k_offset)));
			}
		}
	}
	return out;
}

void TrackedBox::applyEvidence(const Eigen::MatrixXf& corr, const MatrixXf& obsPts) {
	vector<btVector3> estPos(m_nNodes), estVel(m_nNodes), relPos(m_nNodes);
	estPos = getPoints();

	for (int i=0; i < m_nNodes; ++i)  {
		relPos[i] = estPos[i] - getSim()->rigidBody->getCenterOfMassPosition();
		estVel[i] = getSim()->rigidBody->getVelocityInLocalPoint(relPos[i]);
		//estVel[i] = getSim()->rigidBody->getLinearVelocity();
	}
	vector<btVector3> impulses = calcImpulsesDamped(estPos, estVel, toBulletVectors(obsPts.leftCols(3)), corr, toVec(m_masses), TrackingConfig::kp_box, TrackingConfig::kd_box);

	for (int i=0; i<m_nNodes; ++i) {
        getSim()->rigidBody->applyImpulse(impulses[i], relPos[i]);
		//getSim()->rigidBody->applyCentralImpulse(impulses[i]);
	}
}

MatrixXf TrackedBox::extractFeatures(ColorCloudPtr in) {
	MatrixXf out(in->size(), m_nFeatures);
	for (int i=0; i < in->size(); ++i)
		out.row(i) << in->points[i].x, in->points[i].y, in->points[i].z;
	return featuresTransform(out);
}

MatrixXf TrackedBox::getFeatures() {
	MatrixXf features(m_nNodes, m_nFeatures);
	features = toEigenMatrix(getPoints());
	return featuresTransform(features);
}

const VectorXf TrackedBox::getPriorDist() {
	VectorXf prior_dist(m_nFeatures);
	prior_dist << TrackingConfig::pointPriorDist*METERS, TrackingConfig::pointPriorDist*METERS, TrackingConfig::pointPriorDist*METERS;
	return prior_dist;
}
