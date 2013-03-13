#include "RavenGrabMonitor.h"
#include <Eigen/Dense>
#include "simulation/util.h"
#include "utils/config.h"
#include "utils/logging.h"
using namespace std;
using namespace OpenRAVE;
using namespace Eigen;



/** Constructor for grabs for ravens. */
RavensGrab::RavensGrab(btRigidBody* rb, const btTransform& pose,
		btDynamicsWorld* world,  bool leftFinger_, btTransform & offset_) : Grab(rb, pose, world), leftFinger(leftFinger_),offset(offset_) {}


float RavensGrabMonitor::getGripperAngle(RaveRobotObject::Manipulator::Ptr manip) {
	vector<int> indices;
	manip->manip->GetChildDOFIndices(indices);

	vector<dReal> dofs;
	manip->robot->robot->GetDOFValues(dofs, indices);

	return fabs(dofs[0] - dofs[1]);
}

bool RavensGrabMonitor::isClosed(RaveRobotObject::Manipulator::Ptr manip, float closedThreshold) {
	return (getGripperAngle(manip) < closedThreshold);
}

btTransform RavensGrabMonitor::getManipRot() const {
	btTransform trans(m_manip->getTransform());
	trans.setOrigin(btVector3(0, 0, 0));
	return trans;
}

// Finds some innermost point on the gripper
btVector3 RavensGrabMonitor::getInnerPt(bool left) const {
	btTransform trans(m_manip->robot->getLinkTransform(left ? leftFinger : rightFinger));
	// this assumes that the gripper is symmetric when it is closed
	// we get an innermost point on the gripper by transforming a point
	// on the center of the gripper when it is closed
	const btTransform &origInv = left ? origLeftFingerInvTrans : origRightFingerInvTrans;
	return trans * origInv * centerPt;
	// actually above, we can just cache origInv * centerPt
}


// Returns the direction that the specified finger will move when closing
// (manipulator frame)
btVector3 RavensGrabMonitor::getClosingDirection(bool left) const {
	//btTransform trans(m_manip->robot->getLinkTransform(left ? leftFinger : rightFinger));
	return (left? 1 : -1)*m_manip->getTransform().getBasis().getColumn(1);
	//return trans.getBasis().getColumn(col);
}

// Returns the direction that the specified finger will move when closing
// (manipulator frame)
btVector3 RavensGrabMonitor::getToolDirection() const {
	return getManipRot() * btVector3(0,0,1);
}

btTransform RavensGrabMonitor::getInverseFingerTfm (bool left) {
	btVector3 z = getToolDirection();
	btVector3 y = getClosingDirection(left);
	btVector3 x = y.cross(z);

	btVector3 o = getInnerPt(left) + z*0.015*METERS;

	btTransform trans;

	x.normalize();
	y.normalize();
	z.normalize();

	btMatrix3x3 rot;
	rot.setValue(x.getX(), x.getY(), x.getZ(),
			y.getX(), y.getY(), y.getZ(),
			z.getX(), z.getY(), z.getZ());

	trans.setBasis(rot.transpose());
	trans.setOrigin(o);
	return trans.inverse();
}

// Returns true is pt is on the inner side of the specified finger of the gripper
bool RavensGrabMonitor::onInnerSide(const btVector3 &pt, bool left) {
	// then the innerPt and the closing direction define the plane
	btTransform itfm = getInverseFingerTfm (left);
	btVector3 new_pt = itfm * pt;
	return (abs(new_pt.x()) < .0025*METERS) && (new_pt.y() > -.001*METERS) && (new_pt.z() < 0.005*METERS);
}


/** Look for contacts between the specified finger and target
    if the applied impulse reaches some threshold, this returns true
    signifying that the gripper cannot be closed further without penetrating the target. */
bool RavensGrabMonitor::checkContacts(bool left, btRigidBody *target, double &avg_impulse, float threshold) {
	btRigidBody * const finger =
			m_manip->robot->associatedObj(left ? leftFinger : rightFinger)->rigidBody.get();

	const btScalar a = 0.15; // moving average
	const btScalar multiplier = 3.0;
	BulletInstance::Ptr bullet = m_manip->robot->getEnvironment()->bullet;
	for (int i = 0; i < bullet->dispatcher->getNumManifolds(); ++i) {
		btPersistentManifold* contactManifold = bullet->dispatcher->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

		if ((obA != finger || obB != target) && (obB != finger || obA != target)) {
			continue;
		}

		for (int j = 0; j < contactManifold->getNumContacts(); j++) {

			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			btScalar impulse    = pt.getAppliedImpulse();

			cout <<" Found contacts with impulse "<<(float) impulse<<"."<<endl;

			btVector3 contact_pt = pt.getPositionWorldOnA();
			if (avg_impulse <= 0.0001) {
				avg_impulse = (double)impulse;
			}

			btTransform tfm = getInverseFingerTfm(left);

			if (impulse > multiplier*avg_impulse) {
				cout<<"   Found exceeding impulse"<<endl;
				if (onInnerSide(contact_pt, left)) {
					//contact_pt = tfm.inverse()*contact_pt;
					//cout<<"Point: " <<contact_pt.x()<<","<<contact_pt.y()<<","<<contact_pt.z()<<endl;
					//cout<<"Distance: "<<contact_pt.length()<<endl;
					//cout<<"   Found inner contact point"<<endl;
					return true;
				} else {
					//contact_pt = tfm.inverse()*contact_pt;
					//cout<<"Point: " <<contact_pt.x()<<","<<contact_pt.y()<<","<<contact_pt.z()<<endl;
					//cout<<"Distance: "<<contact_pt.length()<<endl;
					cout<<"   NOT Found inner point"<<endl;
				}
			}

			avg_impulse = (double) a*impulse + (1.0 - a)*avg_impulse;
			cout<<"   weighted avg: "<<avg_impulse<<endl;
		}
	}
	return false;
}


RavensGrabMonitor::RavensGrabMonitor(RaveRobotObject::Manipulator::Ptr manip, btDynamicsWorld *dynamicsWorld,
		const string &leftFingerName, const string &rightFingerName, Scene &s_) :
		s(s_),
		numGrabbed(0),
		Monitor(manip),
		m_world(dynamicsWorld),
		m_bodies(),
		m_grabs(),
		m_i(0),
		leftFinger(manip->robot->robot->GetLink(leftFingerName)),
		rightFinger(manip->robot->robot->GetLink(rightFingerName)),
		origLeftFingerInvTrans(manip->robot->getLinkTransform(leftFinger).inverse()),
		origRightFingerInvTrans(manip->robot->getLinkTransform(rightFinger).inverse()),
		centerPt(manip->getTransform().getOrigin()),
		indices() {

	manip->manip->GetChildDOFIndices(indices);
	s.addPreStepCallback(boost::bind(&RavensGrabMonitor::updateGrabPose,this));
}

void RavensGrabMonitor::grab() {grab(100);}

void RavensGrabMonitor::grab(float threshold) {
	// grabs objects in contact
	//cout << "grabbing objects in contact" << endl;

	int num_in_contact = 0;
	for (int i = 0; i < m_bodies.size(); i += 1) { // each m_body is a compound object.

		double avg_impulse = 0.0;
		bool r_contact = false;
		bool l_contact = false;
		for (int j=0; j < m_bodies[i]->children.size(); j+=1) {// for each bullet object in the compound object

			float adjusted_thresh = threshold;
			if (m_bodies[i]->children[j]->objectType() == "BoxObject") adjusted_thresh *= 50;

			// check for contact
			r_contact = checkContacts(false, m_bodies[i]->children[j]->rigidBody.get(), avg_impulse, adjusted_thresh);
			l_contact = checkContacts(true,  m_bodies[i]->children[j]->rigidBody.get(), avg_impulse, adjusted_thresh);


			if (l_contact || r_contact) {
				num_in_contact += 1;
				boost::shared_ptr<btRigidBody>  finger = m_manip->robot->associatedObj(l_contact ? leftFinger : rightFinger)->rigidBody;
				btTransform bodyTfm;
				m_bodies[i]->children[j]->motionState->getWorldTransform(bodyTfm);
				btTransform offset = finger->getWorldTransform().inverseTimes(bodyTfm);
				RavensGrab::Ptr grab(new RavensGrab(m_bodies[i]->children[j]->rigidBody.get(), bodyTfm, m_world, l_contact, offset));
				m_grabs.push_back(grab);
				cout<< " Grabbed : "<<num_in_contact<<" objects."<<endl;
			}
		}
	}
	numGrabbed = num_in_contact;
}


void RavensGrabMonitor::release() {
	numGrabbed = 0;
	while (m_grabs.size() != 0)
		m_grabs.pop_back();
}


void RavensGrabMonitor::updateGrabPose() {
	if (!m_grabs.size()) return;
	for (int i=0; i < m_grabs.size(); i+=1) {
		btTransform targetT;
		m_manip->robot->associatedObj(m_grabs[i]->leftFinger ? leftFinger : rightFinger)->motionState->getWorldTransform(targetT);
		m_grabs[i]->updatePose(targetT*m_grabs[i]->offset);
	}
}
