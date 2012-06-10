#include "tracked_object.h"
#include "config_tracking.h"
#include "utils/conversions.h"
#include "simulation/bullet_io.h"

using namespace std;
using namespace Eigen;

TrackedBox::TrackedBox(BoxObject::Ptr sim) : TrackedObject(sim, "box") {
	m_nEdgeNodes = 4;
	m_nNodes = pow(m_nEdgeNodes, 3) - pow(m_nEdgeNodes-2, 3);
	m_masses.resize(m_nNodes);
	float invMass = getSim()->rigidBody->getInvMass();
	for (int i=0; i < m_nNodes; i++) {
		m_masses(i) = (1/invMass)/m_nNodes;
	}
}

std::vector<btVector3> TrackedBox::getPoints() {
	BoxObject* box_object = dynamic_cast<BoxObject*>(m_sim.get());
	btCollisionShape* bt_collision_shape = dynamic_cast<btCollisionShape*>(box_object->collisionShape.get());
	btBoxShape* bt_box_shape = dynamic_cast<btBoxShape*>(bt_collision_shape);
	btVector3 half_extents = bt_box_shape->getHalfExtentsWithMargin();
	//std::vector<btVector3> out(m_nNodes);
	std::vector<btVector3> out;
	float midNEdgeNodes = ((float) m_nEdgeNodes - 1.0)/2.0;

	btTransform cm = getSim()->rigidBody->getCenterOfMassTransform();
	for (int i=0; i < m_nEdgeNodes; i++) {
		for (int j=0; j < m_nEdgeNodes; j++) {
			for (int k=0; k < m_nEdgeNodes; k++) {
				if (i!=0 && i!=(m_nEdgeNodes-1) && j!=0 && j!=(m_nEdgeNodes-1) && k!=0 && k!=(m_nEdgeNodes-1)) continue;
				float i_offset = ((float) i)/midNEdgeNodes - 1.0;
                float j_offset = ((float) j)/midNEdgeNodes - 1.0;
                float k_offset = ((float) k)/midNEdgeNodes - 1.0;
				//out[(int)pow(m_nEdgeNodes, 2.0)*i+m_nEdgeNodes*j+k] = cm.getOrigin() + cm.getBasis() * (half_extents * btVector3(i_offset, j_offset, k_offset));
                out.push_back(cm.getOrigin() + cm.getBasis() * (half_extents * btVector3(i_offset, j_offset, k_offset)));
			}
		}
	}
	return out;
}

void TrackedBox::applyEvidence(const SparseMatrixf& corr, const MatrixXf& obsPts) {
	vector<btVector3> estPos(m_nNodes), estVel(m_nNodes), relPos(m_nNodes);
	estPos = getPoints();

	for (int i=0; i < m_nNodes; ++i)  {
		relPos[i] = estPos[i] - getSim()->rigidBody->getCenterOfMassPosition();
		estVel[i] = getSim()->rigidBody->getVelocityInLocalPoint(relPos[i]);
		//estVel[i] = getSim()->rigidBody->getLinearVelocity();
	}
	vector<btVector3> impulses = calcImpulsesDamped(estPos, estVel, toBulletVectors(obsPts), corr, toVec(m_masses), TrackingConfig::kp_box, TrackingConfig::kd_box);

	for (int i=0; i<m_nNodes; ++i) {
        getSim()->rigidBody->applyImpulse(impulses[i], relPos[i]);
		//getSim()->rigidBody->applyCentralImpulse(impulses[i]);
	}
}
