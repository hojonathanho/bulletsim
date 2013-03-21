#include "RavenGrabMonitor.h"
#include "CustomScene.h"
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

/*
 * Transform at the middle of the gripper.
 * Origin close to middle of gripper.
 * z points outward in pointing direction
 * y points in closing direction of finger
 * x points out of plane of gripper
 */
btTransform RavensGrabMonitor::getInverseFingerTfm (bool left) {
	btVector3 z = -1*getToolDirection();
	btVector3 y = getClosingDirection(left);
	btVector3 x = y.cross(z);

	btVector3 o = getInnerPt(left) + z*0.003*METERS;

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
	return ((abs(new_pt.x()) < .0025*METERS) && (new_pt.y() > -.001*METERS) && (abs(new_pt.z()) < 0.005*METERS));
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

			//if (impulse > 100000) return true;

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
					cout<<"-------"<<endl;
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


RavensGrabMonitor::RavensGrabMonitor(RaveRobotObject::Manipulator::Ptr _manip, btDynamicsWorld *dynamicsWorld,
		char _gripper, const string &leftFingerName, const string &rightFingerName, CustomScene &s_) :
		gripper (_gripper),
		s(s_),
		numGrabbed(0),
		Monitor(_manip),
		m_world(dynamicsWorld),
		m_bodies(),
		m_grabs(),
		m_i(0),
		leftFinger(_manip->robot->robot->GetLink(leftFingerName)),
		rightFinger(_manip->robot->robot->GetLink(rightFingerName)),
		origLeftFingerInvTrans(_manip->robot->getLinkTransform(leftFinger).inverse()),
		origRightFingerInvTrans(_manip->robot->getLinkTransform(rightFinger).inverse()),
		centerPt(_manip->getTransform().getOrigin()),
		indices() {

	manip = (gripper == 'l' ? s.ravens.manipL : s.ravens.manipR);
	manip->manip->GetChildDOFIndices(indices);
	s.addPreStepCallback(boost::bind(&RavensGrabMonitor::updateGrabPose,this));
}

void RavensGrabMonitor::grab() {grab(false);}

void RavensGrabMonitor::grab(bool grabN, float threshold) {
	// grabs objects in contact
	//cout << "grabbing objects in contact" << endl;

	int num_in_contact = 0;
	for (int i = 0; i < m_bodies.size(); i += 1) { // each m_body is a compound object.

		double avg_impulse = 0.0;
		bool r_contact = false;
		bool l_contact = false;
		for (int j=0; j < m_bodies[i]->children.size(); j+=1) {// for each bullet object in the compound object


			if (grabN && (s.sNeedle->s_needle->children[0]->isKinematic || s.sNeedle->s_needle_mass == 0)
					&& m_bodies[i]->objectType() == "RaveObject") {
				// To get to the center of the gripper
				btTransform gpTfm = manip->getTransform();
				btVector3 offset = gpTfm.getBasis().getColumn(2)*-0.003*METERS;

				btVector3 eePose = manip->getTransform().getOrigin() + offset;
				if (s.sNeedle->pointCloseToNeedle(eePose)) {
					grabNeedle();
					cout<<"Grabbed needle!"<<endl;
					num_in_contact += 1;
				}
				continue;
			}

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


// Reorients and grabs needle
void RavensGrabMonitor::grabNeedle () {
	s.sNeedle->s_grasping_gripper = gripper;
	s.sNeedle->s_gripperManip = manip;


	// ********* Adjust position of needle to be properly in gripper ********//
	btTransform gpTfm = manip->getTransform();
	btVector3 offset = gpTfm.getBasis().getColumn(2)*-0.003*METERS;
	btVector3 eePose = manip->getTransform().getOrigin() + offset;

	btTransform tfm = s.sNeedle->getNeedleCenterTransform();
	btVector3 center = tfm.getOrigin();
	btVector3 xVec = tfm.getBasis().getColumn(0);
	btVector3 zVec = tfm.getBasis().getColumn(2);

	// Vector from center to point being checked
	btVector3 dVec = eePose-center;
	// Vector component out of plane of needle
	btVector3 dVecZ = dVec.dot(zVec)*zVec;
	// Vector in the plane of needle being checked for distance.
	btVector3 dVecXY = dVec - dVecZ;

	btTransform nTfm = s.sNeedle->s_needle->getIndexTransform(0);

	// Move needle to inside gripper
	nTfm.getOrigin() += zVec + (dVecXY.length()-s.sNeedle->s_needle_radius*METERS)*dVecXY.normalized();

	if (s.sNeedle->s_needle->children[0]->isKinematic)
		s.sNeedle->s_needle->getChildren()[0]->motionState->setKinematicPos(nTfm);
	else if (s.sNeedle->s_needle_mass == 0)
		s.sNeedle->s_needle->getChildren()[0]->motionState->setWorldTransform(nTfm);

	/*************************************************************************/


	// Find proper needle tfm relative to gripper
	btTransform wFee = manip->getTransform(), wFn = s.sNeedle->s_needle->getIndexTransform(0);
	btTransform eeFn = wFee.inverse()*wFn;
	s.sNeedle->s_grasp_tfm = eeFn;

	// grab needle for openrave - not sure if this is useful
	vector<KinBody::LinkPtr> links;
	manip->manip->GetChildLinks(links);
	s.ravens.ravens->robot->Release(s.sNeedle->s_needle->body);
	s.ravens.ravens->robot->Grab(s.sNeedle->s_needle->body, links[0]);
}


void RavensGrabMonitor::release() {
	numGrabbed = 0;
	while (m_grabs.size() != 0)
		m_grabs.pop_back();
	if (s.sNeedle->s_grasping_gripper == gripper) {
		s.sNeedle->s_grasping_gripper = 'n';
		btTransform current = s.sNeedle->s_needle->getIndexTransform(0);
		current.getOrigin().setZ(s.table->getIndexTransform(0).getOrigin().z()+s.table->getHalfExtents().z() + 0.005*METERS);
		if (s.sNeedle->s_needle->children[0]->isKinematic)
			s.sNeedle->s_needle->getChildren()[0]->motionState->setKinematicPos(current);
		else if (s.sNeedle->s_needle_mass == 0)
			s.sNeedle->s_needle->getChildren()[0]->motionState->setWorldTransform(current);
		s.ravens.ravens->robot->Release(s.sNeedle->s_needle->body);
	}
}


void RavensGrabMonitor::updateGrabPose() {
	if (!m_grabs.size()) return;
	for (int i=0; i < m_grabs.size(); i+=1) {
		btTransform targetT;
		m_manip->robot->associatedObj(m_grabs[i]->leftFinger ? leftFinger : rightFinger)->motionState->getWorldTransform(targetT);
		m_grabs[i]->updatePose(targetT*m_grabs[i]->offset);
	}
}
