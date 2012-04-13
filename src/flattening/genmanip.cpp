#include "genmanip.h"
#include "simulation/config_bullet.h"

RaveRobotObject::Manipulator::Ptr GenManip::baseManip() const {
    switch (type) {
    case GenManip::TYPE_FAKE:
        return manip.fake->m_manip;
    case GenManip::TYPE_IK:
        return manip.ik;
    }
    cout << "error: gen manip type " << type << " not recognized by GenManip::baseManip()" << endl;
    return RaveRobotObject::Manipulator::Ptr();
}

btTransform GenManip::getTransform() const {
    switch (type) {
    case GenManip::TYPE_FAKE:
        return manip.fake->getTransform();
    case GenManip::TYPE_IK:
        return manip.ik->getTransform();
    }
    cout << "error: gen manip type " << type << " not recognized by GenManip::getTransform()" << endl;
    return btTransform::getIdentity();
}

GenManip::Ptr GenManip::getForked(Fork &f, RaveRobotObject::Ptr forkRobot) const {
    switch (type) {
    case GenManip::TYPE_FAKE: {
        TelekineticGripper::Ptr forkg = boost::static_pointer_cast<TelekineticGripper>(f.forkOf(manip.fake));
        BOOST_ASSERT(forkg);
        return Ptr(new GenManip(forkg));
    }
    case GenManip::TYPE_IK:
        return Ptr(new GenManip(forkRobot->getManipByIndex(manip.ik->index)));
    }
    cout << "error: gen manip type " << type << " not recognized by GenManip::getForked()" << endl;
    return Ptr();
}

btTransform GenManip::getLinkTransform(KinBody::LinkPtr link) const {
    switch (type) {
    case GenManip::TYPE_FAKE:
        return manip.fake->getLinkTransform(link);
    case GenManip::TYPE_IK:
        return manip.ik->robot->getLinkTransform(link);
    }
    cout << "error: gen manip type " << type << " not recognized by GenManip::getLinkTransform()" << endl;
    return btTransform::getIdentity();
}

BulletObject::Ptr GenManip::getLinkRigidBody(KinBody::LinkPtr link) const {
    switch (type) {
    case GenManip::TYPE_FAKE:
        return manip.fake->getLinkRigidBody(link);
    case GenManip::TYPE_IK:
        cout << "return real link rigid body" << endl;
        return manip.ik->robot->associatedObj(link);
    }
    cout << "error: gen manip type " << type << " not recognized by GenManip::getLinkRigidBody()" << endl;
    return BulletObject::Ptr();
}


static const char LEFT_GRIPPER_LEFT_FINGER_NAME[] = "l_gripper_l_finger_tip_link";
static const char LEFT_GRIPPER_RIGHT_FINGER_NAME[] = "l_gripper_r_finger_tip_link";
static const char RIGHT_GRIPPER_LEFT_FINGER_NAME[] = "r_gripper_l_finger_tip_link";
static const char RIGHT_GRIPPER_RIGHT_FINGER_NAME[] = "r_gripper_r_finger_tip_link";

static inline btTransform getManipRot(GenManip::Ptr gmanip) {
    btTransform trans(gmanip->getTransform());
    trans.setOrigin(btVector3(0, 0, 0));
    return trans;
}
// Returns the direction that the specified finger will move when closing
// (manipulator frame)
static inline btVector3 getClosingDirection(GenManip::Ptr gmanip, bool left) {
    // points straight down in the PR2 initial position (manipulator frame)
    btVector3 toolDir = util::toBtVector(gmanip->baseManip()->manip->GetLocalToolDirection());
    // vector normal to the direction that the gripper fingers move in the manipulator frame
    // (on the PR2 this points back into the arm)
    btVector3 closingNormal(
        gmanip->baseManip()->manip->GetClosingDirection()[0],
        gmanip->baseManip()->manip->GetClosingDirection()[1],
        gmanip->baseManip()->manip->GetClosingDirection()[2]);
    return (left ? 1 : -1) * toolDir.cross(closingNormal);
}
// Finds some innermost point on the gripper
static inline btVector3 getInnerPt(GenManip::Ptr gmanip, KinBody::LinkPtr leftFinger, KinBody::LinkPtr rightFinger, bool left) {
    btTransform trans(gmanip->getLinkTransform(left ? leftFinger : rightFinger));
    // we get an innermost point on the gripper by transforming a point
    // on the center of the gripper when it is closed
    return trans * (METERS/20.*btVector3((left ? 1 : -1) * 0.234402, -0.299, 0));
}

// Returns true is pt is on the inner side of the specified finger of the gripper
static inline bool onInnerSide(GenManip::Ptr gmanip, const btVector3 &pt, KinBody::LinkPtr leftFinger, KinBody::LinkPtr rightFinger, bool left) {
    // then the innerPt and the closing direction define the plane
    return (getManipRot(gmanip) * getClosingDirection(gmanip, left)).dot(pt - getInnerPt(gmanip, leftFinger, rightFinger, left)) > 0;
}

static bool inGraspRegion(GenManip::Ptr gmanip, const btVector3 &pt, KinBody::LinkPtr leftFinger, KinBody::LinkPtr rightFinger) {
    // extra padding for more anchors (for stability)
    static const float TOLERANCE = 0.00;

    // check that pt is between the fingers
    if (!onInnerSide(gmanip, pt, leftFinger, rightFinger, true)
            || !onInnerSide(gmanip, pt, leftFinger, rightFinger, false))
        return false;

    // check that pt is behind the gripper tip
    btVector3 x = gmanip->getTransform().inverse() * pt;
    if (x.z() > gmanip->baseManip()->robot->scale*(0.02 + TOLERANCE)) return false;

    // check that pt is within the finger width
    if (abs(x.x()) > gmanip->baseManip()->robot->scale*(0.01 + TOLERANCE)) return false;

    cout << "ATTACHING: " << x.x() << ' ' << x.y() << ' ' << x.z() << endl;

    return true;
}

GenPR2SoftGripper::GenPR2SoftGripper(
        RaveRobotObject::Ptr robot_,
        GenManip::Ptr gmanip_, bool leftGripper) :
    robot(robot_), gmanip(gmanip_),
    leftFinger(robot_->robot->GetLink(leftGripper ? LEFT_GRIPPER_LEFT_FINGER_NAME : RIGHT_GRIPPER_LEFT_FINGER_NAME)),
    rightFinger(robot_->robot->GetLink(leftGripper ? LEFT_GRIPPER_RIGHT_FINGER_NAME : RIGHT_GRIPPER_RIGHT_FINGER_NAME)),
    grabOnlyOnContact(false)
{
}

void GenPR2SoftGripper::releaseAllAnchors() {
    for (int i = 0; i < anchors.size(); ++i)
        sb->removeAnchor(anchors[i]);
    anchors.clear();
    grabbing = false;
}

void GenPR2SoftGripper::grab() {
    attach(true);
    grabbing = true;
}

void GenPR2SoftGripper::attach(bool left) {
    btRigidBody *rigidBody =
        gmanip->getLinkRigidBody(left ? leftFinger : rightFinger)->rigidBody.get();

    set<const btSoftBody::Node*> attached;

    // look for nodes in gripper region
    const btSoftBody::tNodeArray &nodes = sb->softBody->m_nodes;
    for (int i = 0; i < nodes.size(); ++i) {
        if (inGraspRegion(gmanip, nodes[i].m_x, leftFinger, rightFinger) && !sb->hasAnchorAttached(i)) {
            anchors.push_back(sb->addAnchor(i, rigidBody));
            attached.insert(&nodes[i]);
        }
    }

    // look for faces with center in gripper region
    const btSoftBody::tFaceArray &faces = sb->softBody->m_faces;
    for (int i = 0; i < faces.size(); ++i) {
        btVector3 ctr = (1./3.) * (faces[i].m_n[0]->m_x
                + faces[i].m_n[1]->m_x + faces[i].m_n[2]->m_x);
        if (inGraspRegion(gmanip, ctr, leftFinger, rightFinger)) {
            for (int z = 0; z < 3; ++z) {
                int idx = faces[i].m_n[z] - &nodes[0];
                if (!sb->hasAnchorAttached(idx)) {
                    anchors.push_back(sb->addAnchor(idx, rigidBody));
                    attached.insert(&nodes[idx]);
                }
            }
        }
    }

    // now for each added anchor, add anchors to neighboring nodes for stability
    const int MAX_EXTRA_ANCHORS = 3;
    const btSoftBody::tLinkArray &links = sb->softBody->m_links;
    const int origNumAnchors = anchors.size();
    for (int i = 0; i < links.size(); ++i) {
        if (anchors.size() >= origNumAnchors + MAX_EXTRA_ANCHORS)
            break;
        if (attached.find(links[i].m_n[0]) != attached.end()) {
            int idx = links[i].m_n[1] - &nodes[0];
            if (!sb->hasAnchorAttached(idx)) {
                anchors.push_back(sb->addAnchor(idx, rigidBody));
            }
        }
        else if (attached.find(links[i].m_n[1]) != attached.end()) {
            int idx = links[i].m_n[0] - &nodes[0];
            if (!sb->hasAnchorAttached(idx)) {
                anchors.push_back(sb->addAnchor(idx, rigidBody));
            }
        }
    }

    cout << "appended " << attached.size() << " anchors to " << (left ? "left" : "right") << endl;
}
