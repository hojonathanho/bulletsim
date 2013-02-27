#ifndef _PR2_SOFTY_
#define _PR2_SOFTY_

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/util.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <boost/shared_ptr.hpp>

// I've only tested this on the PR2 model
class SoftBodyGripperAction : public Action {
    RaveRobotObject::Manipulator::Ptr manip;
    vector<dReal> startVals;
    dReal endVal;
    vector<int> indices;
    vector<dReal> vals;

    // min/max gripper dof vals
    static const float CLOSED_VAL = 0.0f, OPEN_VAL = 0.25f;

    KinBody::LinkPtr leftFinger, rightFinger;
    const btTransform origLeftFingerInvTrans, origRightFingerInvTrans;

    // the point right where the fingers meet when the gripper is closed
    // (in the robot's initial pose)
    const btVector3 centerPt;

    // the target softbody
    btSoftBody *psb;
    BulletSoftObject::Ptr softBody;

    // appended anchors
    vector<BulletSoftObject::AnchorHandle> anchors;

    btTransform getManipRot() const {
        btTransform trans(manip->getTransform());
        trans.setOrigin(btVector3(0, 0, 0));
        return trans;
    }

    // Finds some innermost point on the gripper
    btVector3 getInnerPt(bool left) const {
        btTransform trans(manip->robot->getLinkTransform(left ? leftFinger : rightFinger));
        // this assumes that the gripper is symmetric when it is closed
        // we get an innermost point on the gripper by transforming a point
        // on the center of the gripper when it is closed
        const btTransform &origInv = left ? origLeftFingerInvTrans : origRightFingerInvTrans;
        return trans * origInv * centerPt;
        // actually above, we can just cache origInv * centerPt
    }


    // Returns the direction that the specified finger will move when closing
    // (manipulator frame)
    btVector3 getClosingDirection(bool left) const {
    	btTransform trans(manip->robot->getLinkTransform(left ? leftFinger : rightFinger));
    	return trans.getBasis().getColumn(2);
    }

    // Returns true is pt is on the inner side of the specified finger of the gripper
    bool onInnerSide(const btVector3 &pt, bool left) const {
        // then the innerPt and the closing direction define the plane
    	btVector3 z = getManipRot() * btVector3(0,0,1);
    	btVector3 o = getInnerPt(left) + z*0.01*GeneralConfig::scale;

    	return getClosingDirection(left).dot(pt - getInnerPt(left)) > -.005*GeneralConfig::scale  &&
    		   ((pt-getInnerPt(left)).dot(z) < 0);
    }

    // Fills in the rcontacs array with contact information between psb and pco
    static void getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts) {
        // custom contact checking adapted from btSoftBody.cpp and btSoftBodyInternals.h
        struct Custom_CollideSDF_RS : btDbvt::ICollide {
            Custom_CollideSDF_RS(btSoftBody::tRContactArray &rcontacts_) : rcontacts(rcontacts_) { }

            void Process(const btDbvtNode* leaf) {
                btSoftBody::Node* node=(btSoftBody::Node*)leaf->data;
                DoNode(*node);
            }

            void DoNode(btSoftBody::Node& n) {
                const btScalar m=n.m_im>0?dynmargin:stamargin;
                btSoftBody::RContact c;
                if (!n.m_battach && psb->checkContact(m_colObj1,n.m_x,m,c.m_cti)) {
                    const btScalar  ima=n.m_im;
                    const btScalar  imb= m_rigidBody? m_rigidBody->getInvMass() : 0.f;
                    const btScalar  ms=ima+imb;
                    if(ms>0) {
                        // there's a lot of extra information we don't need to compute
                        // since we just want to find the contact points
#if 0
                        const btTransform&      wtr=m_rigidBody?m_rigidBody->getWorldTransform() : m_colObj1->getWorldTransform();
                        static const btMatrix3x3        iwiStatic(0,0,0,0,0,0,0,0,0);
                        const btMatrix3x3&      iwi=m_rigidBody?m_rigidBody->getInvInertiaTensorWorld() : iwiStatic;
                        const btVector3         ra=n.m_x-wtr.getOrigin();
                        const btVector3         va=m_rigidBody ? m_rigidBody->getVelocityInLocalPoint(ra)*psb->m_sst.sdt : btVector3(0,0,0);
                        const btVector3         vb=n.m_x-n.m_q;
                        const btVector3         vr=vb-va;
                        const btScalar          dn=btDot(vr,c.m_cti.m_normal);
                        const btVector3         fv=vr-c.m_cti.m_normal*dn;
                        const btScalar          fc=psb->m_cfg.kDF*m_colObj1->getFriction();
#endif
                        c.m_node        =       &n;
#if 0
                        c.m_c0          =       ImpulseMatrix(psb->m_sst.sdt,ima,imb,iwi,ra);
                        c.m_c1          =       ra;
                        c.m_c2          =       ima*psb->m_sst.sdt;
                        c.m_c3          =       fv.length2()<(btFabs(dn)*fc)?0:1-fc;
                        c.m_c4          =       m_colObj1->isStaticOrKinematicObject()?psb->m_cfg.kKHR:psb->m_cfg.kCHR;
#endif
                        rcontacts.push_back(c);
#if 0
                        if (m_rigidBody)
                                m_rigidBody->activate();
#endif
                    }
                }
            }
            btSoftBody*             psb;
            btCollisionObject*      m_colObj1;
            btRigidBody*    m_rigidBody;
            btScalar                dynmargin;
            btScalar                stamargin;
            btSoftBody::tRContactArray &rcontacts;
        };

        Custom_CollideSDF_RS  docollide(rcontacts);
        btRigidBody*            prb1=btRigidBody::upcast(pco);
        btTransform     wtr=pco->getWorldTransform();

        const btTransform       ctr=pco->getWorldTransform();
        const btScalar          timemargin=(wtr.getOrigin()-ctr.getOrigin()).length();
        const btScalar          basemargin=psb->getCollisionShape()->getMargin();
        btVector3                       mins;
        btVector3                       maxs;
        ATTRIBUTE_ALIGNED16(btDbvtVolume)               volume;
        pco->getCollisionShape()->getAabb(      pco->getWorldTransform(),
                mins,
                maxs);
        volume=btDbvtVolume::FromMM(mins,maxs);
        volume.Expand(btVector3(basemargin,basemargin,basemargin));
        docollide.psb           =       psb;
        docollide.m_colObj1 = pco;
        docollide.m_rigidBody = prb1;

        docollide.dynmargin     =       basemargin+timemargin;
        docollide.stamargin     =       basemargin;
        psb->m_ndbvt.collideTV(psb->m_ndbvt.m_root,volume,docollide);
    }

    // Checks if psb is touching the inside of the gripper fingers
    // If so, attaches anchors to every contact point
    void attach() {
        btRigidBody *rigidBody_r =
            manip->robot->associatedObj(rightFinger)->rigidBody.get();
        btRigidBody *rigidBody_l =
                    manip->robot->associatedObj(leftFinger)->rigidBody.get();

        btSoftBody::tRContactArray rcontacts_r;
        btSoftBody::tRContactArray rcontacts_l;
        getContactPointsWith(psb, rigidBody_r, rcontacts_r);
        getContactPointsWith(psb, rigidBody_l, rcontacts_l);
        cout << "got r " << rcontacts_r.size() << " contacts\n";
        cout << "got l " << rcontacts_l.size() << " contacts\n";
        for (int i = 0; i < rcontacts_r.size(); ++i) {
            const btSoftBody::RContact &c = rcontacts_r[i];
            KinBody::LinkPtr colLink = manip->robot->associatedObj(c.m_cti.m_colObj);
            if (!colLink) continue;
            const btVector3 &contactPt = c.m_node->m_x;
            if (onInnerSide(contactPt, left) && onInnerSide(contactPt, right)) {
                cout << "\tappending anchor\n";
                anchors.push_back(softBody->addAnchor(c.m_node, rigidBody_r));
            }
        }

        for (int i = 0; i < rcontacts_l.size(); ++i) {
        	const btSoftBody::RContact &c = rcontacts_l[i];
        	KinBody::LinkPtr colLink = manip->robot->associatedObj(c.m_cti.m_colObj);
        	if (!colLink) continue;
        	const btVector3 &contactPt = c.m_node->m_x;
        	if (onInnerSide(contactPt, left) && onInnerSide(contactPt, right)) {
        		cout << "\tappending anchor\n";
        		anchors.push_back(softBody->addAnchor(c.m_node, rigidBody_l));
        	}
        }
    }

public:
    typedef boost::shared_ptr<SoftBodyGripperAction> Ptr;
    SoftBodyGripperAction(RaveRobotObject::Manipulator::Ptr manip_,
                  const string &leftFingerName,
                  const string &rightFingerName,
                  float time) :
            Action(time), manip(manip_), vals(2, 0),
            leftFinger(manip->robot->robot->GetLink(leftFingerName)),
            rightFinger(manip->robot->robot->GetLink(rightFingerName)),
            origLeftFingerInvTrans(manip->robot->getLinkTransform(leftFinger).inverse()),
            origRightFingerInvTrans(manip->robot->getLinkTransform(rightFinger).inverse()),
            centerPt(manip->getTransform().getOrigin()),
            indices()
    {
    	manip->manip->GetChildDOFIndices(indices);
        setCloseAction();
    }

    btVector3 getVec1 (bool left) const {
    	return getInnerPt(left);
    }

    btVector3 getVec2 (bool left) const {
    	btVector3 z = getManipRot() * btVector3(0,0,1);
    	btVector3 o = getInnerPt(left) + z*0.01*GeneralConfig::scale;
    	return o;
    }

    void setEndpoints(vector<dReal> start, dReal end) { startVals = start; endVal = end; }

    vector<dReal> getCurrDOFVal() const {
        vector<dReal> v;
        manip->robot->robot->GetDOFValues(v, indices);
        return v;
    }
    void setOpenAction() { setEndpoints(getCurrDOFVal(), OPEN_VAL); }
    void setCloseAction() { setEndpoints(getCurrDOFVal(), CLOSED_VAL); }
    void toggleAction() {
        if (endVal == CLOSED_VAL)
            setOpenAction();
        else if (endVal == OPEN_VAL)
            setCloseAction();
    }

    // Must be called before the action is run!
    void setTarget(BulletSoftObject::Ptr sb) {
        softBody = sb;
        psb = sb->softBody.get();
    }

    void releaseAllAnchors() {
        for (int i = 0; i < anchors.size(); ++i)
            softBody->removeAnchor(anchors[i]);
        anchors.clear();
    }

    void reset() {
        Action::reset();
        releaseAllAnchors();
    }

    void step(float dt) {
        if (done()) return;
        stepTime(dt);

        float frac = fracElapsed();
        vals[0] = (1.f - frac)*startVals[0] + frac*endVal;
        vals[1] = (1.f - frac)*startVals[1] + frac*-1*endVal;

        manip->robot->setDOFValues(indices, vals);

        if (vals[0] == CLOSED_VAL && vals[1] == CLOSED_VAL) {
            attach();
        }
    }
};

#endif
