#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <openrave/kinbody.h>
#include "robots/pr2.h"
#include <Eigen/Dense>
#include <Eigen/SVD>

//#define USE_PR2

//class RigidMover
//{
//    public:
//    RigidMover(BulletObject::Ptr pobject_in, btVector3 pos, btDynamicsWorld* world){
//        pobject = pobject_in;
//        btRigidBody* rb = pobject->rigidBody.get();
//        cnt = new btGeneric6DofConstraint(*rb,btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)),true); // second parameter?
//        cnt->setLinearLowerLimit(btVector3(0,0,0));
//        cnt->setLinearUpperLimit(btVector3(0,0,0));
//        cnt->setAngularLowerLimit(btVector3(0,0,0));
//        cnt->setAngularUpperLimit(btVector3(0,0,0));
//        world->addConstraint(cnt);
//        updatePosition(pos);
//    }

//    typedef boost::shared_ptr<RigidMover> Ptr;

//    void updatePosition(btVector3 pos) {
//      cnt->getFrameOffsetA().setOrigin(pos);
//    }

//    btTransform GetTransform() { return cnt->getFrameOffsetA();}
//    void SetTransform(btTransform tm) { cnt->getFrameOffsetA() = tm;}

//    BulletObject::Ptr pobject;
//    btGeneric6DofConstraint* cnt;
//};


//class CapsuleRope : public CompoundObject<BulletObject> {
//private:
//  float angStiffness;
//  float angDamping;
//  float linDamping;
//  float angLimit;
//public:
//  typedef boost::shared_ptr<CapsuleRope> Ptr;
//  std::vector<boost::shared_ptr<btCollisionShape> > shapes;
//  std::vector<BulletConstraint::Ptr> joints;
//  btScalar radius;
//  int nLinks;

//  CapsuleRope(const std::vector<btVector3>& ctrlPoints, float radius_, float angStiffness_=.1, float angDamping_=1, float linDamping_=.75, float angLimit_=.4);
//  void init();
//  void destroy();
//  std::vector<btVector3> getNodes();
//  std::vector<btVector3> getControlPoints();
//};


class GripperKinematicObject : public CompoundObject<BoxObject>{
public:
    float apperture;
    btTransform cur_tm;
    bool bOpen;
    bool bAttached;
    //BoxObject::Ptr top_jaw, bottom_jaw;


    typedef boost::shared_ptr<GripperKinematicObject> Ptr;

//    GripperKinematicObject(const GripperKinematicObject& gripper);
    GripperKinematicObject();
    void translate(btVector3 transvec);
    void setWorldTransform(btTransform tm);
    btTransform getWorldTransform(){return cur_tm;}
    void getWorldTransform(btTransform& in){in = cur_tm;}
    void toggle();
    void toggleattach(btSoftBody * psb);
    void getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts);
    void appendAnchor(btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence=1);
    void releaseAllAnchors(btSoftBody * psb) {psb->m_anchors.clear();}


//    EnvironmentObject::Ptr copy(Fork &f) const {
//        CompoundObject<BoxObject>::Ptr o(new CompoundObject<BoxObject>());
//        internalCopy(o, f);
//        return o;
//    }

//    void internalCopy(CompoundObject<BulletObject>::Ptr o, Fork &f) const {
//        o->children.reserve(children.size());
//         ChildVector::const_iterator i;
//        for (i = children.begin(); i != children.end(); ++i) {
//            if (*i)
//                o->children.push_back(boost::static_pointer_cast<BoxObject> ((*i)->copy(f)));
//            else
//                o->children.push_back( BoxObject::Ptr());
//        }
//    }


    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new GripperKinematicObject());
        //printf("copying gripper\n");
        internalCopy(o, f);
        return o;
    }
    void internalCopy(GripperKinematicObject::Ptr o, Fork &f) const {
        o->apperture = apperture;
        o->cur_tm = cur_tm;
        o->bOpen = bOpen;
        o->bAttached = bAttached;

        o->children.clear();
        o->children.reserve(children.size());
        ChildVector::const_iterator i;
        for (i = children.begin(); i != children.end(); ++i) {
            if (*i)
                o->children.push_back(boost::static_pointer_cast<BoxObject> ((*i)->copy(f)));
            else
                o->children.push_back(BoxObject::Ptr());
        }

        //o->top_jaw = boost::static_pointer_cast<BoxObject> (children[0]);
        //o->bottom_jaw = boost::static_pointer_cast<BoxObject> (children[1]);

        //f.registerCopy(top_jaw->rigidBody.get(), o->top_jaw->rigidBody.get());
        //f.registerCopy(bottom_jaw->rigidBody.get(), o->bottom_jaw->rigidBody.get());
    }

};


//GrabberKinematicObject::GrabberKinematicObject(float halfExtents_) :
//    halfExtents(halfExtents_), height(height_),
//    BulletObject(0, new btBoxShape(halfExtents_),
//            btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)), true) {
//}


void GripperKinematicObject::translate(btVector3 transvec)
{
    btTransform tm = getWorldTransform();
    tm.setOrigin(tm.getOrigin() + transvec);
    setWorldTransform(tm);
}

void GripperKinematicObject::toggleattach(btSoftBody * psb) {

    if(bAttached)
        releaseAllAnchors(psb);
    else
    {
        for(int k = 0; k < 2; k++)
        {
            BoxObject::Ptr part;
            if(k == 0)
                part = children[0];
            else
                part = children[1];

            btRigidBody* rigidBody = part->rigidBody.get();
            btSoftBody::tRContactArray rcontacts;
            getContactPointsWith(psb, rigidBody, rcontacts);
            cout << "got " << rcontacts.size() << " contacts\n";

            //if no contacts, return without toggling bAttached
            if(rcontacts.size() == 0)
                return;

            for (int i = 0; i < rcontacts.size(); ++i) {
                const btSoftBody::RContact &c = rcontacts[i];
                //btRigidBody* colLink = c.m_cti.m_colObj;
                //if (!colLink) continue;
                const btVector3 &contactPt = c.m_node->m_x;
                //if (onInnerSide(contactPt, left)) {
                    appendAnchor(psb, c.m_node, rigidBody);
                    cout << "\tappending anchor\n";
            }
        }
    }
    bAttached = !bAttached;
}

// Fills in the rcontacs array with contact information between psb and pco
void GripperKinematicObject::getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts) {
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
                    c.m_node        =       &n;

                    rcontacts.push_back(c);

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

// adapted from btSoftBody.cpp (btSoftBody::appendAnchor)
void GripperKinematicObject::appendAnchor(btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence) {
    btSoftBody::Anchor a;
    a.m_node = node;
    a.m_body = body;
    a.m_local = body->getWorldTransform().inverse()*a.m_node->m_x;
    a.m_node->m_battach = 1;
    a.m_influence = influence;
    psb->m_anchors.push_back(a);
}



//GripperKinematicObject::GripperKinematicObject(const GripperKinematicObject& gripper)
//{

//}

GripperKinematicObject::GripperKinematicObject()
{
    bAttached = false;
    apperture = 4;
    BoxObject::Ptr top_jaw(new BoxObject(0, btVector3(.75,.75,0.2),btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, apperture/2)),true));
    top_jaw->setColor(1,0,0,1);
    BoxObject::Ptr bottom_jaw(new BoxObject(0, btVector3(.75,.75,.2),btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,0,-apperture/2)),true));
    bottom_jaw->setColor(0,0,1,1);
    top_jaw->motionState->getWorldTransform(cur_tm);
    cur_tm.setOrigin(cur_tm.getOrigin() - btVector3(0,0,-apperture/2));
    bOpen = true;
    children.push_back(top_jaw);
    children.push_back(bottom_jaw);
}

void GripperKinematicObject::setWorldTransform(btTransform tm)
{

    btTransform top_tm = tm;
    btTransform bottom_tm = tm;

    btTransform top_offset;

    children[0]->motionState->getWorldTransform(top_offset);
    top_offset = cur_tm.inverse()*top_offset;

    top_tm.setOrigin(top_tm.getOrigin() + top_tm.getBasis().getColumn(2)*(top_offset.getOrigin()[2]));
    bottom_tm.setOrigin(bottom_tm.getOrigin() - bottom_tm.getBasis().getColumn(2)*(top_offset.getOrigin()[2]));

    children[0]->motionState->setKinematicPos(top_tm);
    children[1]->motionState->setKinematicPos(bottom_tm);

    cur_tm = tm;
}

void GripperKinematicObject::toggle()
{

    btTransform top_tm;
    btTransform bottom_tm;
    children[0]->motionState->getWorldTransform(top_tm);
    children[1]->motionState->getWorldTransform(bottom_tm);


    if(bOpen)
    {

        btTransform top_offset = cur_tm.inverse()*top_tm;

        float close_length = 1.01*top_offset.getOrigin()[2] - children[0]->halfExtents[2];

        top_tm.setOrigin(top_tm.getOrigin() - close_length*top_tm.getBasis().getColumn(2));
        bottom_tm.setOrigin(bottom_tm.getOrigin() + close_length*bottom_tm.getBasis().getColumn(2));

    }
    else
    {
        top_tm.setOrigin(top_tm.getOrigin() + top_tm.getBasis().getColumn(2)*(apperture/2));
        bottom_tm.setOrigin(bottom_tm.getOrigin() - bottom_tm.getBasis().getColumn(2)*(apperture/2));

    }

    children[0]->motionState->setKinematicPos(top_tm);
    children[1]->motionState->setKinematicPos(bottom_tm);


    bOpen = !bOpen;

}


// I've only tested this on the PR2 model
class PR2SoftBodyGripperAction : public Action {
    RaveRobotObject::Manipulator::Ptr manip;
    dReal startVal, endVal;
    vector<int> indices;
    vector<dReal> vals;

    // min/max gripper dof vals
    static const float CLOSED_VAL = 0.03f, OPEN_VAL = 0.54f;

    KinBody::LinkPtr leftFinger, rightFinger;
    const btTransform origLeftFingerInvTrans, origRightFingerInvTrans;

    // the point right where the fingers meet when the gripper is closed
    // (in the robot's initial pose)
    const btVector3 centerPt;

    // vector normal to the direction that the gripper fingers move in the manipulator frame
    // (on the PR2 this points back into the arm)
    const btVector3 closingNormal;

    // points straight down in the PR2 initial position (manipulator frame)
    const btVector3 toolDirection;

    // the target softbody
    btSoftBody *psb;

    btTransform getManipRot() const {
        btTransform trans(manip->getTransform());
        trans.setOrigin(btVector3(0, 0, 0));
        return trans;
    }

    // Returns the direction that the specified finger will move when closing
    // (manipulator frame)
    btVector3 getClosingDirection(bool left) const {
        return (left ? 1 : -1) * toolDirection.cross(closingNormal);
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

    // Returns true is pt is on the inner side of the specified finger of the gripper
    bool onInnerSide(const btVector3 &pt, bool left) const {
        // then the innerPt and the closing direction define the plane
        return (getManipRot() * getClosingDirection(left)).dot(pt - getInnerPt(left)) > 0;
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

    // adapted from btSoftBody.cpp (btSoftBody::appendAnchor)
    static void appendAnchor(btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence=1) {
        btSoftBody::Anchor a;
        a.m_node = node;
        a.m_body = body;
        a.m_local = body->getWorldTransform().inverse()*a.m_node->m_x;
        a.m_node->m_battach = 1;
        a.m_influence = influence;
        psb->m_anchors.push_back(a);
    }

    // Checks if psb is touching the inside of the gripper fingers
    // If so, attaches anchors to every contact point
    void attach(bool left) {
        btRigidBody *rigidBody =
            manip->robot->associatedObj(left ? leftFinger : rightFinger)->rigidBody.get();
        btSoftBody::tRContactArray rcontacts;
        getContactPointsWith(psb, rigidBody, rcontacts);
        cout << "got " << rcontacts.size() << " contacts\n";
        for (int i = 0; i < rcontacts.size(); ++i) {
            const btSoftBody::RContact &c = rcontacts[i];
            KinBody::LinkPtr colLink = manip->robot->associatedObj(c.m_cti.m_colObj);
            if (!colLink) continue;
            const btVector3 &contactPt = c.m_node->m_x;
            if (onInnerSide(contactPt, left)) {
                appendAnchor(psb, c.m_node, rigidBody);
                cout << "\tappending anchor\n";
            }
        }
    }

public:
    typedef boost::shared_ptr<PR2SoftBodyGripperAction> Ptr;
    PR2SoftBodyGripperAction(RaveRobotObject::Manipulator::Ptr manip_,
                  const string &leftFingerName,
                  const string &rightFingerName,
                  float time) :
            Action(time), manip(manip_), vals(1, 0),
            leftFinger(manip->robot->robot->GetLink(leftFingerName)),
            rightFinger(manip->robot->robot->GetLink(rightFingerName)),
            origLeftFingerInvTrans(manip->robot->getLinkTransform(leftFinger).inverse()),
            origRightFingerInvTrans(manip->robot->getLinkTransform(rightFinger).inverse()),
            centerPt(manip->getTransform().getOrigin()),
            indices(manip->manip->GetGripperIndices()),
            closingNormal(manip->manip->GetClosingDirection()[0],
                          manip->manip->GetClosingDirection()[1],
                          manip->manip->GetClosingDirection()[2]),
            toolDirection(util::toBtVector(manip->manip->GetLocalToolDirection())) // don't bother scaling
    {
        if (indices.size() != 1)
            cout << "WARNING: more than one gripper DOF; just choosing first one" << endl;
        setCloseAction();
    }

    void setEndpoints(dReal start, dReal end) { startVal = start; endVal = end; }
    dReal getCurrDOFVal() const {
        vector<dReal> v;
        manip->robot->robot->GetDOFValues(v);
        return v[indices[0]];
    }
    void setOpenAction() { setEndpoints(getCurrDOFVal(), OPEN_VAL); } // 0.54 is the max joint value for the pr2
    void setCloseAction() { setEndpoints(getCurrDOFVal(), CLOSED_VAL); }
    void toggleAction() {
        if (endVal == CLOSED_VAL)
            setOpenAction();
        else if (endVal == OPEN_VAL)
            setCloseAction();
    }

    // Must be called before the action is run!
    void setTarget(btSoftBody *psb_) { psb = psb_; }

    void releaseAllAnchors() {
        psb->m_anchors.clear();
    }

    void reset() {
        Action::reset();
        releaseAllAnchors();
    }

    void step(float dt) {
        if (done()) return;
        stepTime(dt);

        float frac = fracElapsed();
        vals[0] = (1.f - frac)*startVal + frac*endVal;
        manip->robot->setDOFValues(indices, vals);

        if (vals[0] == CLOSED_VAL) {
            attach(true);
            attach(false);
        }
    }
};


struct CustomScene : public Scene {
#ifdef USE_PR2
    PR2SoftBodyGripperAction::Ptr leftAction, rightAction;
#else


    GripperKinematicObject::Ptr left_gripper, right_gripper, left_gripper_orig, right_gripper_orig, left_gripper_fork, right_gripper_fork;

    struct {
        bool transGrabber0,rotateGrabber0,transGrabber1,rotateGrabber1, startDragging;
        float dx, dy, lastX, lastY;
    } inputState;

#endif
    BulletSoftObject::Ptr clothptr, clothptr_orig, clothptr_fork;
    BulletInstance::Ptr bullet2;
    OSGInstance::Ptr osg2;
    Fork::Ptr fork;
    RaveRobotObject::Ptr origRobot, tmpRobot;




#ifdef USE_PR2
    PR2Manager pr2m;

    CustomScene() : pr2m(*this) { }
#else
    CustomScene(){
        inputState.transGrabber0 =  inputState.rotateGrabber0 =  inputState.transGrabber1 =  inputState.rotateGrabber1 =  inputState.startDragging = false;

        left_gripper_orig.reset(new GripperKinematicObject());
        left_gripper_orig->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,-10,0)));
        env->add(left_gripper_orig);

        right_gripper_orig.reset(new GripperKinematicObject());
        right_gripper_orig->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,10,0)));
        env->add(right_gripper_orig);

        left_gripper = left_gripper_orig;
        right_gripper = right_gripper_orig;

        fork.reset();
    }

#endif

    BulletSoftObject::Ptr createCloth(btScalar s, const btVector3 &center);
    void createFork();
    void destroyFork();
    void swapFork();
    Eigen::MatrixXf computeJacobian();

    void run();
};


Eigen::MatrixXf CustomScene::computeJacobian()
{
    //printf("starting jacobian computation\n");
    //stopLoop();
    bool bBackupLoopState = loopState.skip_step;
    loopState.skip_step = true;

    int numnodes = clothptr->softBody->m_nodes.size();
    Eigen::VectorXf  V_before(numnodes*3);
    Eigen::VectorXf  V_after(V_before);

    for(int k = 0; k < numnodes; k++)
    {
        for(int j = 0; j < 3; j++)
            V_before(3*k + j) = clothptr->softBody->m_nodes[k].m_x[j];
    }

    Eigen::MatrixXf J(numnodes*3,3);

    std::vector<btVector3> perts;
    float step_length = 0.2;
    perts.push_back(btVector3(step_length,0,0));
    perts.push_back(btVector3(0,step_length,0));
    perts.push_back(btVector3(0,0,step_length));

    for(int i = 0; i < 3 ; i++)
    {
        createFork();
        swapFork(); //now pointers are set to the forked objects

        //apply perturbation

//        btTransform tm;
//        left_gripper->getWorldTransform(tm);
//        tm.setOrigin(tm.getOrigin() + perts[i]);
//        left_gripper->setWorldTransform(tm);

        left_gripper->translate(perts[i]);

        //stepFor(BulletConfig::dt, 0.5);

        float time = 0.05;

        while (time > 0) {
            int maxsteps= BulletConfig::maxSubSteps;
            float internaldt = BulletConfig::internalTimeStep;
            float startTime=viewer.getFrameStamp()->getSimulationTime(), endTime;

            if (syncTime && drawingOn)
                endTime = viewer.getFrameStamp()->getSimulationTime();

            // run pre-step callbacks
            for (int i = 0; i < prestepCallbacks.size(); ++i)
                prestepCallbacks[i]();

            //env->step(dt, maxsteps, internaldt);
            for (std::set<Fork::Ptr>::iterator i = forks.begin(); i != forks.end(); ++i)
                (*i)->env->step(BulletConfig::dt, maxsteps, internaldt);

            draw();

            if (syncTime && drawingOn) {
                float timeLeft = BulletConfig::dt - (endTime - startTime);
                idleFor(timeLeft);
                startTime = endTime + timeLeft;
            }
            time -= BulletConfig::dt;
        }

        for(int k = 0; k < numnodes; k++)
        {
            for(int j = 0; j < 3; j++)
                V_after(3*k + j) = clothptr->softBody->m_nodes[k].m_x[j];
        }

        destroyFork();
        J.col(i) = (V_after - V_before)/step_length;
    }

    //cout << J<< endl;


    loopState.skip_step = bBackupLoopState;
    //printf("done jacobian computation\n");
    return J;
}

//template<typename Scalar>
//bool pinv(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &a, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &a_pinv)
//{
//    // see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method

//    if ( a.rows()<a.cols() )
//        return false;

//    // SVD
//    Eigen::SVD< Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > svdA(a);

//    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> vSingular = svdA.singularValues();

//    // Build a diagonal matrix with the Inverted Singular values
//    // The pseudo inverted singular matrix is easy to compute :
//    // is formed by replacing every nonzero entry by its reciprocal (inversing).
//    Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Eigen::RowMajor> vPseudoInvertedSingular(svdA.matrixV().cols(),1);

//    for (int iRow =0; iRow<vSingular.rows(); iRow++)
//    {
//        if ( fabs(vSingular(iRow))<=1e-10 ) // Todo : Put epsilon in parameter
//        {
//            vPseudoInvertedSingular(iRow,0)=0.;
//        }
//        else
//        {
//            vPseudoInvertedSingular(iRow,0)=1./vSingular(iRow);
//        }
//    }

//    // A little optimization here
//    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mAdjointU = svdA.matrixU().adjoint().block(0,0,vSingular.rows(),svdA.matrixU().adjoint().cols());

//    // Pseudo-Inversion : V * S * U'
//    a_pinv = (svdA.matrixV() *  vPseudoInvertedSingular.asDiagonal()) * mAdjointU  ;

//    return true;
//}

class CustomKeyHandler : public osgGA::GUIEventHandler {
    CustomScene &scene;
public:
    CustomKeyHandler(CustomScene &scene_) : scene(scene_) { }
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
};

bool CustomKeyHandler::handle(const osgGA::GUIEventAdapter &ea,osgGA::GUIActionAdapter & aa) {
    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case 'f':
            scene.createFork();
            break;
        case 'g':
            {
            scene.swapFork();
            }
            break;

        case 'h':
            scene.destroyFork();
            break;

        case '1':
            scene.inputState.transGrabber0 = true; break;
        case 'q':
            scene.inputState.rotateGrabber0 = true; break;

        case '2':
            scene.inputState.transGrabber1 = true; break;
        case 'w':
            scene.inputState.rotateGrabber1 = true; break;
#ifdef USE_PR2
        case 'a':
            scene.leftAction->reset();
            scene.leftAction->toggleAction();
            scene.runAction(scene.leftAction, BulletConfig::dt);

            break;
        case 's':

            scene.rightAction->reset();
            scene.rightAction->toggleAction();
            scene.runAction(scene.rightAction, BulletConfig::dt);

            break;
#else
        case 'a':
            scene.left_gripper->toggle();
            break;

        case 's':
            scene.right_gripper->toggle();
            break;

        case 'z':
            scene.left_gripper->toggleattach(scene.clothptr->softBody.get());
            break;

        case 'x':
            scene.right_gripper->toggleattach(scene.clothptr->softBody.get());
            break;

//        case 'b':
//            btVector3 probe_move2 = btVector3(0,0,0.2);
//            btTransform tm2;
//            scene.left_grabber->motionState->getWorldTransform(tm2);
//            printf("origin: %f %f %f\n",tm2.getOrigin()[0],tm2.getOrigin()[1],tm2.getOrigin()[2]);
//            tm2.setOrigin(tm2.getOrigin() + probe_move2);
//            printf("neworigin: %f %f %f\n",tm2.getOrigin()[0],tm2.getOrigin()[1],tm2.getOrigin()[2]);
//            //scene.left_grabber->motionState->setWorldTransform(tm2);
//            scene.left_grabber->motionState->setKinematicPos(tm2);
//            break;
        }
        break;

    case osgGA::GUIEventAdapter::KEYUP:
        switch (ea.getKey()) {
        case '1':
            scene.inputState.transGrabber0 = false; break;
        case 'q':
            scene.inputState.rotateGrabber0 = false; break;
        case '2':
            scene.inputState.transGrabber1 = false; break;
        case 'w':
            scene.inputState.rotateGrabber1 = false; break;

        case 'j':
            {
                scene.loopState.skip_step = true;
                int numnodes = scene.clothptr->softBody->m_nodes.size();

                Eigen::VectorXf nodestep(3);
                nodestep(0) = -1;
                nodestep(1) = 0;
                nodestep(2) = 0;

                Eigen::VectorXf V_step(numnodes*3);


                for(int k = 0; k < numnodes; k++)
                {
                    for(int j = 0; j < 3; j++)
                        V_step(3*k + j) = nodestep(j);
                }

                Eigen::VectorXf V_trans;
                for(int i = 0; i < 100; i++)
                {
                    Eigen::MatrixXf Jt(scene.computeJacobian().transpose());
                    V_trans = Jt*V_step;
                    V_trans = V_trans/V_trans.norm()*0.2;
                    cout << "trans vec: " << V_trans.transpose() << endl;
                    btVector3 transvec(V_trans(0),V_trans(1),V_trans(2));
                    scene.left_gripper->translate(transvec);
                    scene.stepFor(BulletConfig::dt, 0.2);
                }


                scene.loopState.skip_step = false;
            }

        }
        break;

    case osgGA::GUIEventAdapter::PUSH:
        scene.inputState.startDragging = true;
        break;

    case osgGA::GUIEventAdapter::DRAG:
        if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG){
            // drag the active manipulator in the plane of view
            if ( (ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON) &&
                  (scene.inputState.transGrabber0 || scene.inputState.transGrabber1 ||
                   scene.inputState.rotateGrabber0 || scene.inputState.rotateGrabber1)) {
                if (scene.inputState.startDragging) {
                    scene.inputState.dx = scene.inputState.dy = 0;
                } else {
                    scene.inputState.dx = scene.inputState.lastX - ea.getXnormalized();
                    scene.inputState.dy = ea.getYnormalized() - scene.inputState.lastY;
                }
                scene.inputState.lastX = ea.getXnormalized(); scene.inputState.lastY = ea.getYnormalized();
                scene.inputState.startDragging = false;

                // get our current view
                osg::Vec3d osgCenter, osgEye, osgUp;
                scene.manip->getTransformation(osgCenter, osgEye, osgUp);
                btVector3 from(util::toBtVector(osgEye));
                btVector3 to(util::toBtVector(osgCenter));
                btVector3 up(util::toBtVector(osgUp)); up.normalize();

                // compute basis vectors for the plane of view
                // (the plane normal to the ray from the camera to the center of the scene)
                btVector3 normal = (to - from).normalized();
                btVector3 yVec = (up - (up.dot(normal))*normal).normalized(); //FIXME: is this necessary with osg?
                btVector3 xVec = normal.cross(yVec);
                btVector3 dragVec = SceneConfig::mouseDragScale*10 * (scene.inputState.dx*xVec + scene.inputState.dy*yVec);
                //printf("dx: %f dy: %f\n",scene.inputState.dx,scene.inputState.dy);

                btTransform origTrans;
                if (scene.inputState.transGrabber0 || scene.inputState.rotateGrabber0)
                {
                    //scene.left_grabber->motionState->getWorldTransform(origTrans);
                    scene.left_gripper->getWorldTransform(origTrans);
                    //origTrans = scene.left_grabber->rigidBody->getCenterOfMassTransform();
                }
                else
                {
                    scene.right_gripper->getWorldTransform(origTrans);
                    //scene.right_grabber->motionState->getWorldTransform(origTrans);
                }


                //printf("origin: %f %f %f\n",origTrans.getOrigin()[0],origTrans.getOrigin()[1],origTrans.getOrigin()[2]);

                btTransform newTrans(origTrans);

                if (scene.inputState.transGrabber0 || scene.inputState.transGrabber1)
                    // if moving the manip, just set the origin appropriately
                    newTrans.setOrigin(dragVec + origTrans.getOrigin());
                else if (scene.inputState.rotateGrabber0 || scene.inputState.rotateGrabber1) {
                    // if we're rotating, the axis is perpendicular to the
                    // direction the mouse is dragging
                    btVector3 axis = normal.cross(dragVec);
                    btScalar angle = dragVec.length();
                    btQuaternion rot(axis, angle);
                    // we must ensure that we never get a bad rotation quaternion
                    // due to really small (effectively zero) mouse movements
                    // this is the easiest way to do this:
                    if (rot.length() > 0.99f && rot.length() < 1.01f)
                        newTrans.setRotation(rot * origTrans.getRotation());
                }
                //printf("newtrans: %f %f %f\n",newTrans.getOrigin()[0],newTrans.getOrigin()[1],newTrans.getOrigin()[2]);
                //softbody ->addForce(const btVector3& forceVector,int node)
                if (scene.inputState.transGrabber0 || scene.inputState.rotateGrabber0)
                {
                    //scene.left_grabber->motionState->setKinematicPos(newTrans);
                    scene.left_gripper->setWorldTransform(newTrans);

                }
                else
                {
                    scene.right_gripper->setWorldTransform(newTrans);
                    //scene.right_grabber->motionState->setKinematicPos(newTrans);
                }
                return true;
            }
        }
        break;
#endif
    }
    return false;
}

BulletSoftObject::Ptr CustomScene::createCloth(btScalar s, const btVector3 &center) {
    const int divs = 45;

    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        env->bullet->softBodyWorldInfo,
        center + btVector3(-s,-s,0),
        center + btVector3(+s,-s,0),
        center + btVector3(-s,+s,0),
        center + btVector3(+s,+s,0),
        divs, divs,
        0, true);

    psb->m_cfg.piterations = 10;//2;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
        | btSoftBody::fCollision::CL_RS
        | btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 1.0;
    psb->getCollisionShape()->setMargin(0.05);
    btSoftBody::Material *pm = psb->appendMaterial();
    pm->m_kLST = 0.2;//0.1;
    psb->m_cfg.kDP = 0.05;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    psb->setTotalMass(1, true);
    psb->generateClusters(0);

/*    for (int i = 0; i < psb->m_clusters.size(); ++i) {
        psb->m_clusters[i]->m_selfCollisionImpulseFactor = 0.1;
    }*/

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

void CustomScene::createFork() {
    if(fork)
    {
        destroyFork();
    }
    bullet2.reset(new BulletInstance);
    bullet2->setGravity(BulletConfig::gravity);
    osg2.reset(new OSGInstance);
    osg->root->addChild(osg2->root.get());

    fork.reset(new Fork(env, bullet2, osg2));
    registerFork(fork);

    //cout << "forked!" << endl;

#ifdef USE_PR2
    origRobot = pr2m.pr2;
    EnvironmentObject::Ptr p = fork->forkOf(pr2m.pr2);
    if (!p) {
        cout << "failed to get forked version of robot!" << endl;
        return;
    }
    tmpRobot = boost::static_pointer_cast<RaveRobotObject>(p);
    cout << (tmpRobot->getEnvironment() == env.get()) << endl;
    cout << (tmpRobot->getEnvironment() == fork->env.get()) << endl;
#else
    left_gripper_fork = boost::static_pointer_cast<GripperKinematicObject> (fork->forkOf(left_gripper));
    right_gripper_fork = boost::static_pointer_cast<GripperKinematicObject> (fork->forkOf(right_gripper));
    clothptr_fork = boost::static_pointer_cast<BulletSoftObject> (fork->forkOf(clothptr));
#endif
}

void CustomScene::destroyFork() {
    if(left_gripper.get() == left_gripper_fork.get())
    {
        left_gripper = left_gripper_orig;
        right_gripper = right_gripper_orig;
        clothptr = clothptr_orig;
    }

    unregisterFork(fork);
    osg->root->removeChild(osg2->root.get());
    fork.reset();
    left_gripper_fork.reset();
    right_gripper_fork.reset();
    clothptr_fork.reset();
}

void CustomScene::swapFork() {
#ifdef USE_PR2
    // swaps the forked robot with the real one
    cout << "swapping!" << endl;
    int leftidx = pr2m.pr2Left->index;
    int rightidx = pr2m.pr2Right->index;
    origRobot.swap(tmpRobot);
    pr2m.pr2 = origRobot;
    pr2m.pr2Left = pr2m.pr2->getManipByIndex(leftidx);
    pr2m.pr2Right = pr2m.pr2->getManipByIndex(rightidx);
#else
    if(left_gripper.get() == left_gripper_orig.get())
    {
        left_gripper = left_gripper_fork;
        right_gripper = right_gripper_fork;
        clothptr = clothptr_fork;
    }
    else
    {
        left_gripper = left_gripper_orig;
        right_gripper = right_gripper_orig;
        clothptr = clothptr_orig;
    }

#endif

/*    vector<int> indices; vector<dReal> vals;
    for (int i = 0; i < tmpRobot->robot->GetDOF(); ++i) {
        indices.push_back(i);
        vals.push_back(0);
    }
    tmpRobot->setDOFValues(indices, vals);*/
}



void CustomScene::run() {
    viewer.addEventHandler(new CustomKeyHandler(*this));

    const float dt = BulletConfig::dt;
    const float table_height = .5;
    const float table_thickness = .05;
    BoxObject::Ptr table(
        new BoxObject(0, GeneralConfig::scale * btVector3(.75,.75,table_thickness/2),
            btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(1.2, 0, table_height-table_thickness/2))));
    table->rigidBody->setFriction(1);

    BulletSoftObject::Ptr cloth(
            createCloth(GeneralConfig::scale * 0.25, GeneralConfig::scale * btVector3(0.9, 0, table_height+0.01)));


    btSoftBody* psb = cloth->softBody.get();
    clothptr = clothptr_orig = cloth;
    psb->setTotalMass(0.1);

#ifdef USE_PR2
    pr2m.pr2->ignoreCollisionWith(psb);
#endif


    env->add(table);
    env->add(cloth);

    //left_mover.reset(new RigidMover(table, table->rigidBody->getCenterOfMassPosition(), env->bullet->dynamicsWorld));

#ifdef USE_PR2
    leftAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Left, "l_gripper_l_finger_tip_link", "l_gripper_r_finger_tip_link", 1));
    leftAction->setTarget(psb);
    rightAction.reset(new PR2SoftBodyGripperAction(pr2m.pr2Right, "r_gripper_l_finger_tip_link", "r_gripper_r_finger_tip_link", 1));
    rightAction->setTarget(psb);
#else


    //btVector3 pos(0,0,0);
    //grab_left.reset(new Grab(psb, &psb->m_nodes[0], left_grabber->rigidBody.get()));
    //grab_left.reset(new Grab(psb, 0, left_grabber->rigidBody.get()));

    //psb->m_cfg.kAHR = 1;
    //psb->appendAnchor(0,left_grabber->rigidBody.get());
    //psb->appendAnchor(1,left_grabber->rigidBody.get());
    //psb->appendAnchor(2,left_grabber->rigidBody.get());


    int min_x_ind = -1;
    int max_x_ind = -1;
    int min_y_ind = -1;
    int max_y_ind = -1;
    double min_x = 100;
    double max_x = -100;
    double min_y = 100;
    double max_y = -100;

    for(int i = 0; i < psb->m_nodes.size();i++)
    {
        //printf("%f\n", psb->m_nodes[i].m_x[0]);
        double new_x = psb->m_nodes[i].m_x[0];
        double new_y = psb->m_nodes[i].m_x[1];
        if(new_x > max_x)
        {
            max_x = new_x;
            max_x_ind = i;
        }

        if(new_x < min_x)
        {
            min_x = new_x;
            min_x_ind = i;
        }

        if(new_y > max_y)
        {
            max_y = new_y;
            max_y_ind = i;
        }

        if(new_y < min_y)
        {
            min_y = new_y;
            min_y_ind = i;
        }

    }

    //btTransform tm_left = btTransform(btQuaternion( 0,    0.9877,    0.1564 ,   0), psb->m_nodes[min_x_ind].m_x+btVector3(0,0,2));
    //btTransform tm_right = btTransform(btQuaternion( 0,    0.9877,    0.1564 ,   0), psb->m_nodes[max_x_ind].m_x+btVector3(0,0,2));
    //left_grabber->motionState->setKinematicPos(tm_left);
    //right_grabber->motionState->setKinematicPos(tm_right);

    //psb->appendAnchor(min_x_ind,left_grabber->rigidBody.get());
    //psb->appendAnchor(max_x_ind,right_grabber->rigidBody.get());

    btTransform tm_left = btTransform(btQuaternion( 0,    0,    0 ,   1), psb->m_nodes[min_x_ind].m_x);
    left_gripper->setWorldTransform(tm_left);
    //left_gripper->toggle();


    btTransform tm_right = btTransform(btQuaternion( 0,    0,    0 ,   1), psb->m_nodes[max_x_ind].m_x);
    right_gripper->setWorldTransform(tm_right);
    //right_gripper->toggle();

    //psb->appendAnchor(min_x_ind,left_gripper->top_jaw->rigidBody.get());
    //psb->appendAnchor(max_x_ind,right_gripper->top_jaw->rigidBody.get());




    //    btSoftBody::Anchor a;
//    a.m_node = &psb->m_nodes[0];
//    a.m_body = left_grabber->rigidBody.get();
//    a.m_local = left_grabber->rigidBody.get()->getWorldTransform().inverse()*a.m_node->m_x;
//    a.m_node->m_battach = 1;
//    a.m_influence = 1;
//    psb->m_anchors.push_back(a);


    //grab_left = g;
    //grab_left.updatePosition(ropePtr->bodies[0]->getCenterOfMassPosition() + btVector3(0.5,0,0.5));


#endif

//    btVector3 probe_move = btVector3(0,0,0.05);
//    btTransform tm = left_mover->pobject->rigidBody->getWorldTransform();
//    //btVector3 neworigin = tm.getOrigin() + probe_move;
//    tm.setOrigin(tm.getOrigin() + probe_move);
//    printf("neworigin: %f %f %f\n",tm.getOrigin()[0],tm.getOrigin()[1],tm.getOrigin()[2]);
//    left_mover->SetTransform(tm);


    //setSyncTime(true);
    startViewer();
    stepFor(dt, 2);

    /*
    leftAction->setOpenAction();
    runAction(leftAction, dt);

    rightAction->setOpenAction();
    runAction(rightAction, dt);
    */

    startFixedTimestepLoop(dt);
}

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = 0.01;
    BulletConfig::internalTimeStep = 0.01;
    BulletConfig::maxSubSteps = 0;

    Parser parser;

    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);


    CustomScene().run();
    return 0;
}
