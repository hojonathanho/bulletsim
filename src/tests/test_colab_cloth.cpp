#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <openrave/kinbody.h>
#include "robots/pr2.h"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <omp.h>
#include <boost/date_time/posix_time/posix_time.hpp>


//#define USE_PR2
#define DO_ROTATION

class GripperKinematicObject : public CompoundObject<BoxObject>{
public:
    float apperture;
    btTransform cur_tm;
    bool bOpen;
    bool bAttached;

    typedef boost::shared_ptr<GripperKinematicObject> Ptr;

    GripperKinematicObject(btVector4 color = btVector4(0,0,1,0.3));
    void translate(btVector3 transvec);
    void applyTransform(btTransform tm);
    void setWorldTransform(btTransform tm);
    btTransform getWorldTransform(){return cur_tm;}
    void getWorldTransform(btTransform& in){in = cur_tm;}
    void toggle();
    void toggleattach(btSoftBody * psb);
    void getContactPointsWith(btSoftBody *psb, btCollisionObject *pco, btSoftBody::tRContactArray &rcontacts);
    void appendAnchor(btSoftBody *psb, btSoftBody::Node *node, btRigidBody *body, btScalar influence=1);
    void releaseAllAnchors(btSoftBody * psb) {psb->m_anchors.clear();}

    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new GripperKinematicObject());
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
    }

};


void GripperKinematicObject::applyTransform(btTransform tm)
{

    //btTransform _tm = getWorldTransform();
    //cout << "tm: " << _tm.getOrigin()[0] << " " << _tm.getOrigin()[1] << " " << _tm.getOrigin()[2] << endl;
    //btTransform a = _tm*tm;
    //cout << "tm: " << a.getOrigin()[0] << " " << a.getOrigin()[1] << " " <<  a.getOrigin()[2] << endl;
    setWorldTransform(getWorldTransform()*tm);
}

void GripperKinematicObject::translate(btVector3 transvec)
{
    btTransform tm = getWorldTransform();
    tm.setOrigin(tm.getOrigin() + transvec);
    setWorldTransform(tm);
}

void GripperKinematicObject::toggleattach(btSoftBody * psb) {

    if(bAttached)
    {
        btAlignedObjectArray<btSoftBody::Anchor> newanchors;
        for(int i = 0; i < psb->m_anchors.size(); i++)
        {
            if(psb->m_anchors[i].m_body != children[0]->rigidBody.get() && psb->m_anchors[i].m_body != children[1]->rigidBody.get())
                newanchors.push_back(psb->m_anchors[i]);
        }
        releaseAllAnchors(psb);
        for(int i = 0; i < newanchors.size(); i++)
        {
            psb->m_anchors.push_back(newanchors[i]);
        }
    }
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
                continue;

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




GripperKinematicObject::GripperKinematicObject(btVector4 color)
{
    bAttached = false;
    apperture = 4;
    btVector3 halfextents = btVector3(.3,.3,0.1);
    BoxObject::Ptr top_jaw(new BoxObject(0, halfextents, btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, apperture/2)),true));
    top_jaw->setColor(color[0],color[1],color[2],color[3]);
    BoxObject::Ptr bottom_jaw(new BoxObject(0, halfextents, btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,0,-apperture/2)),true));
    bottom_jaw->setColor(color[0],color[1],color[2],color[3]);
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

class PointReflector{
public:
    PointReflector(float _mid_x, float _min_y, float _max_y)
    {
        mid_x = _mid_x;
        min_y = _min_y;
        max_y = _max_y;
    }

    btVector3 reflect(btVector3& vector_in)
    {
        btVector3 v_out = vector_in;
        v_out[0] = vector_in[0] - 2*(vector_in[0] - mid_x);
        return v_out;
    }

    float mid_x, min_y, max_y;

    typedef boost::shared_ptr<PointReflector> Ptr;
};

struct StepState {
    BulletInstance::Ptr bullet;
    OSGInstance::Ptr osg;
    Fork::Ptr fork;
    BulletSoftObject::Ptr cloth;
    GripperKinematicObject::Ptr gripper;
};


struct CustomScene : public Scene {
#ifdef USE_PR2
    PR2SoftBodyGripperAction::Ptr leftAction, rightAction;
#else
    GripperKinematicObject::Ptr left_gripper, right_gripper, left_gripper_orig, right_gripper_orig, left_gripper_fork, right_gripper_fork;
    GripperKinematicObject::Ptr fixed_gripper1, fixed_gripper2;
    struct {
        bool transGrabber0,rotateGrabber0,transGrabber1,rotateGrabber1, startDragging;
        float dx, dy, lastX, lastY;
    } inputState;

#endif
    bool bTracking, bInTrackingLoop;
    PointReflector::Ptr point_reflector;
    BulletSoftObject::Ptr clothptr, clothptr_orig, clothptr_fork;
    BulletInstance::Ptr bullet2;
    OSGInstance::Ptr osg2;
    Fork::Ptr fork;
    RaveRobotObject::Ptr origRobot, tmpRobot;
    std::map<int, int> node_mirror_map;
    float jacobian_sim_time;
    PlotPoints::Ptr plot_points;
    PlotPoints::Ptr left_center_point;
    PlotAxes::Ptr left_axes;
    //PlotLines::Ptr drag_line;


#ifdef USE_PR2
    PR2Manager pr2m;

    CustomScene() : pr2m(*this) { }
#else
    CustomScene(){
        bTracking = bInTrackingLoop = false;
        inputState.transGrabber0 =  inputState.rotateGrabber0 =  inputState.transGrabber1 =  inputState.rotateGrabber1 =  inputState.startDragging = false;

        jacobian_sim_time = 0.05;

        left_gripper_orig.reset(new GripperKinematicObject());
        left_gripper_orig->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,-10,0)));
        env->add(left_gripper_orig);

        btVector4 color(1,0,0,0.3);
        right_gripper_orig.reset(new GripperKinematicObject(color));
        right_gripper_orig->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,10,0)));
        env->add(right_gripper_orig);

        left_gripper = left_gripper_orig;
        right_gripper = right_gripper_orig;

        color = btVector4(0.6,0.6,0.6,1);
        fixed_gripper1.reset(new GripperKinematicObject(color));
        fixed_gripper1->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,20,0)));
        env->add(fixed_gripper1);
        fixed_gripper2.reset(new GripperKinematicObject(color));
        fixed_gripper2->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,-20,0)));
        env->add(fixed_gripper2);


        fork.reset();
    }

#endif

    BulletSoftObject::Ptr createCloth(btScalar s, const btVector3 &center);
    void createFork();
    void destroyFork();
    void swapFork();
    Eigen::MatrixXf computeJacobian();
    Eigen::MatrixXf computeJacobian_parallel();
    void simulateInNewFork(StepState& innerstate, float sim_time, btTransform& gripper_tm);
    void doJTracking();

    void run();
};


void CustomScene::simulateInNewFork(StepState& innerstate, float sim_time, btTransform& gripper_tm)
{

    innerstate.bullet.reset(new BulletInstance);
    innerstate.bullet->setGravity(BulletConfig::gravity);
    innerstate.osg.reset(new OSGInstance);
    innerstate.fork.reset(new Fork(env, innerstate.bullet, innerstate.osg));
    innerstate.cloth = boost::static_pointer_cast<BulletSoftObject>(innerstate.fork->forkOf(clothptr));
    innerstate.gripper = boost::static_pointer_cast<GripperKinematicObject>(innerstate.fork->forkOf(left_gripper));

    innerstate.gripper->applyTransform(gripper_tm);

    float time = sim_time;
    while (time > 0) {
        // run pre-step callbacks
        //for (int j = 0; j < prestepCallbacks.size(); ++j)
        //    prestepCallbacks[j]();

        innerstate.fork->env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
        time -= BulletConfig::dt;
    }

}


Eigen::MatrixXf CustomScene::computeJacobian_parallel()
{
    boost::posix_time::ptime begTick(boost::posix_time::microsec_clock::local_time());

    //printf("starting jacobian computation\n");
    //stopLoop();
    bool bBackupLoopState = loopState.skip_step;
    loopState.skip_step = true;

    int numnodes = clothptr->softBody->m_nodes.size();
    Eigen::VectorXf  V_before(numnodes*3);


    for(int k = 0; k < numnodes; k++)
    {
        for(int j = 0; j < 3; j++)
            V_before(3*k + j) = clothptr->softBody->m_nodes[k].m_x[j];
    }



    std::vector<btTransform> perts;
    float step_length = 0.2;
    float rot_angle = 0.2;
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(step_length,0,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,step_length,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,step_length)));
#ifdef DO_ROTATION
    perts.push_back(btTransform(btQuaternion(btVector3(1,0,0),rot_angle),btVector3(0,0,0)));
    perts.push_back(btTransform(btQuaternion(btVector3(0,1,0),rot_angle),btVector3(0,0,0)));
    perts.push_back(btTransform(btQuaternion(btVector3(0,0,1),rot_angle),btVector3(0,0,0)));
    omp_set_num_threads(7); //need to find a better way to do this
#else
    omp_set_num_threads(4);
#endif

    Eigen::MatrixXf J(numnodes*3,perts.size());
    #pragma omp parallel shared(J)
    {

        //schedule(static, 1)
        #pragma omp for nowait
        for(int i = 0; i < perts.size(); i++)
        {

            StepState innerstate;
            simulateInNewFork(innerstate, jacobian_sim_time, perts[i]);
            Eigen::VectorXf  V_after(V_before);
            for(int k = 0; k < numnodes; k++)
            {
                for(int j = 0; j < 3; j++)
                    V_after(3*k + j) = innerstate.cloth->softBody->m_nodes[k].m_x[j];
            }
            float divider;
            if(i < 3)
                divider = step_length;
            else
                divider = rot_angle;

            J.col(i) = (V_after - V_before)/divider;
        }
    }

    //cout << J<< endl;
    boost::posix_time::ptime endTick(boost::posix_time::microsec_clock::local_time());
    //std::cout << "time: " << boost::posix_time::to_simple_string(endTick - begTick) << std::endl;

    loopState.skip_step = bBackupLoopState;
    //printf("done jacobian computation\n");
    return J;


}





Eigen::MatrixXf CustomScene::computeJacobian()
{
    boost::posix_time::ptime begTick(boost::posix_time::microsec_clock::local_time());

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

    std::vector<btTransform> perts;
    float step_length = 0.2;
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(step_length,0,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,step_length,0)));
    perts.push_back(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,step_length)));
    float time;

    for(int i = 0; i < 3 ; i++)
    {
        createFork();
        swapFork(); //now pointers are set to the forked objects

        //apply perturbation
        left_gripper->applyTransform(perts[i]);

        time = jacobian_sim_time;
        while (time > 0) {
            float startTime=viewer.getFrameStamp()->getSimulationTime(), endTime;
            if (syncTime && drawingOn)
                endTime = viewer.getFrameStamp()->getSimulationTime();

            // run pre-step callbacks
            //for (int j = 0; j < prestepCallbackssize(); ++j)
            //    prestepCallbacks[j]();

            fork->env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
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
    boost::posix_time::ptime endTick(boost::posix_time::microsec_clock::local_time());
    //std::cout << "time: " << boost::posix_time::to_simple_string(endTick - begTick) << std::endl;


    loopState.skip_step = bBackupLoopState;
    //printf("done jacobian computation\n");
    return J;
}

Eigen::MatrixXf pinv(const Eigen::MatrixXf &a)
{
    // see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method

    if ( a.rows()<a.cols() )
    {
        cout << "pinv error!" << endl;
        return Eigen::MatrixXf();
    }

    // SVD
    Eigen::JacobiSVD< Eigen::MatrixXf> svdA(a,Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXf vSingular = svdA.singularValues();

    // Build a diagonal matrix with the Inverted Singular values
    // The pseudo inverted singular matrix is easy to compute :
    // is formed by replacing every nonzero entry by its reciprocal (inversing).
    Eigen::MatrixXf vPseudoInvertedSingular(svdA.matrixV().cols(),1);

    for (int iRow =0; iRow<vSingular.rows(); iRow++)
    {
        if ( fabs(vSingular(iRow))<=1e-10 ) // Todo : Put epsilon in parameter
        {
            vPseudoInvertedSingular(iRow,0)=0.;
        }
        else
        {
            vPseudoInvertedSingular(iRow,0)=1./vSingular(iRow);
        }
    }

    // A little optimization here
    Eigen::MatrixXf mAdjointU = svdA.matrixU().adjoint().block(0,0,vSingular.rows(),svdA.matrixU().adjoint().cols());

    // Pseudo-Inversion : V * S * U'
    return (svdA.matrixV() *  vPseudoInvertedSingular.asDiagonal()) * mAdjointU  ;

}


void CustomScene::doJTracking()
{

    //this loop is already being executed by someone else, abort
    if(bInTrackingLoop)
        return;
    else
        bInTrackingLoop = true;


    loopState.skip_step = true;
    int numnodes = clothptr->softBody->m_nodes.size();

    float step_limit = 0.2;
    Eigen::VectorXf V_step(numnodes*3);
    float prev_error = 10000000;
    Eigen::VectorXf V_trans;
    //btVector3 transvec;
    btTransform transtm;

    Eigen::MatrixXf J;
    while(bTracking)
    {
        for(int i = 0; i < numnodes*3;i++)
            V_step(i) = 0;

        float error = 0;
        std::vector<btVector3> plotpoints;
        std::vector<btVector4> plotcols;
        for( map<int,int>::iterator ii=node_mirror_map.begin(); ii!=node_mirror_map.end(); ++ii)
        {
            btVector3 targpoint = point_reflector->reflect(clothptr->softBody->m_nodes[(*ii).second].m_x);

            btVector3 targvec = targpoint - clothptr->softBody->m_nodes[(*ii).first].m_x;
            error = error + targvec.length();
            for(int j = 0; j < 3; j++)
                V_step(3*(*ii).first + j) = targvec[j];


            plotpoints.push_back(clothptr->softBody->m_nodes[(*ii).first].m_x);
            plotcols.push_back(btVector4(targvec.length(),0,0,1));
        }
        plot_points->setPoints(plotpoints,plotcols);
        left_axes->setup(left_gripper->getWorldTransform(),1);
        cout << "Error: " << error << " ";


        prev_error = error;
        J = computeJacobian_parallel();
        //Eigen::MatrixXf Jt(J.transpose());
        //V_trans = Jt*V_step;

        Eigen::MatrixXf Jpinv= pinv(J.transpose()*J)*J.transpose();
        V_trans = Jpinv*V_step;

        if(V_trans.norm() > step_limit)
            V_trans = V_trans/V_trans.norm()*step_limit;
        //cout << "trans vec: " << V_trans.transpose() << endl;
        //transvec = btVector3(V_trans(0),V_trans(1),V_trans(2));
#ifdef DO_ROTATION
        transtm = btTransform(btQuaternion(btVector3(0,0,1),V_trans(5))*
                              btQuaternion(btVector3(0,1,0),V_trans(4))*
                              btQuaternion(btVector3(1,0,0),V_trans(3)),
                              btVector3(V_trans(0),V_trans(1),V_trans(2)));
#else
        transtm = btTransform(btQuaternion(0,0,0,1), btVector3(V_trans(0),V_trans(1),V_trans(2)));
#endif
        //check is it more semetric than it would have been had you done nothing
        //simulateInNewFork(innerstate, BulletConfig::dt, transvec);

        float errors[2];
        omp_set_num_threads(2);
        #pragma omp parallel
        {
            #pragma omp for nowait
            for(int i = 0; i < 2 ; i++)
            {
                StepState innerstate;
                //btTransform simtm(btQuaternion(0,0,0,1),transvec);
                btTransform simtm = transtm;
                if(i == 1)
                    simtm = btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));

                simulateInNewFork(innerstate, jacobian_sim_time, simtm);
                errors[i] = 0;
                for( map<int,int>::iterator ii=node_mirror_map.begin(); ii!=node_mirror_map.end(); ++ii)
                {
                    btVector3 targpoint = point_reflector->reflect(innerstate.cloth->softBody->m_nodes[(*ii).second].m_x);

                    btVector3 targvec = targpoint - innerstate.cloth->softBody->m_nodes[(*ii).first].m_x;
                    errors[i] = errors[i] + targvec.length();
                }
            }
        }



        if(errors[0] >= errors[1])
        {
            cout << "Error increase, not moving (new error: " <<  error << ")" << endl;
            //idleFor(0.2);
            //left_gripper->translate(-transvec);
            //stepFor(BulletConfig::dt, 0.2);
            //bTracking = false;
            //stepFor(BulletConfig::dt, jacobian_sim_time);
            break;
        }
        else
        {
            cout << endl;
            left_gripper->applyTransform(transtm);
            //stepFor(BulletConfig::dt, jacobian_sim_time);
        }
        break;

    }

    loopState.skip_step = false;

    bInTrackingLoop = false;
}



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

        case 'b':
            {

            scene.left_gripper->applyTransform(btTransform(btQuaternion(btVector3(0,0,1),0.2),btVector3(0,0,0)));
            scene.left_axes->setup(scene.left_gripper->getWorldTransform(),1);
            }
        }
        break;

    case osgGA::GUIEventAdapter::KEYUP:
        switch (ea.getKey()) {
        case '1':
            scene.inputState.transGrabber0 = false; break;
        case 'q':
            scene.inputState.rotateGrabber0 = false; break;
        case '2':
            scene.inputState.transGrabber1 = false;
            break;
        case 'w':
            scene.inputState.rotateGrabber1 = false;
            break;

        case 'j':
            {
               scene.bTracking = !scene.bTracking;
               if(!scene.bTracking)
                   scene.plot_points->setPoints(std::vector<btVector3> (), std::vector<btVector4> ());
            }
            break;
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
                    scene.left_gripper->getWorldTransform(origTrans);
                }
                else
                {
                    scene.right_gripper->getWorldTransform(origTrans);
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

//                std::vector<btVector3> plot_line;
//                std::vector<btVector4> plot_color;
//                plot_line.push_back(origTrans.getOrigin());
//                plot_line.push_back(origTrans.getOrigin() + 100*(newTrans.getOrigin()- origTrans.getOrigin()));
//                plot_color.push_back(btVector4(1,0,0,1));
//                scene.drag_line->setPoints(plot_line,plot_color);
                if (scene.inputState.transGrabber0 || scene.inputState.rotateGrabber0)
                {
                    scene.left_gripper->setWorldTransform(newTrans);
                }
                else
                {
                    scene.right_gripper->setWorldTransform(newTrans);
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
    //psb->generateClusters(500);

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

    addPreStepCallback(boost::bind(&CustomScene::doJTracking, this));

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

    //std::vector<float> node_x(psb->m_nodes.size());
    //std::vector<float> node_y(psb->m_nodes.size());
    std::vector<btVector3> node_pos(psb->m_nodes.size());
    std::vector<int> corner_ind(4,-1);
    std::vector<btVector3> corner_pnts(4);

    corner_pnts[0] = btVector3(100,100,0);
    corner_pnts[1] = btVector3(100,-100,0);
    corner_pnts[2] = btVector3(-100,100,0);
    corner_pnts[3] = btVector3(-100,-100,0);

    for(int i = 0; i < psb->m_nodes.size();i++)
    {
        //printf("%f\n", psb->m_nodes[i].m_x[0]);
//        double new_x = psb->m_nodes[i].m_x[0];
//        double new_y = psb->m_nodes[i].m_x[1];
        node_pos[i] = psb->m_nodes[i].m_x;

        if(node_pos[i][0] <= corner_pnts[0][0] && node_pos[i][1] <= corner_pnts[0][1])
        {
            corner_ind[0] = i;
            corner_pnts[0] = node_pos[i];
        }

        if(node_pos[i][0] <= corner_pnts[1][0] && node_pos[i][1] >= corner_pnts[1][1])
        {
            corner_ind[1] = i;
            corner_pnts[1] = node_pos[i];
        }

        if(node_pos[i][0] >= corner_pnts[2][0] && node_pos[i][1] <= corner_pnts[2][1])
        {
            corner_ind[2] = i;
            corner_pnts[2] = node_pos[i];
        }

        if(node_pos[i][0] >= corner_pnts[3][0] && node_pos[i][1] >= corner_pnts[3][1])
        {
            corner_ind[3] = i;
            corner_pnts[3] = node_pos[i];
        }

//        if(new_x > max_x)
//        {
//            max_x = new_x;
//            max_x_ind = i;
//        }

//        if(new_x < min_x)
//        {
//            min_x = new_x;
//            min_x_ind = i;
//        }

//        if(new_y > max_y)
//        {
//            max_y = new_y;
//            max_y_ind = i;
//        }

//        if(new_y < min_y)
//        {
//            min_y = new_y;
//            min_y_ind = i;
//        }

    }

//    for(int i = 0; i < 4; i++)
//    {
//        cout << corner_pnts[i][0] << " " << corner_pnts[i][1] << " " << corner_pnts[i][2] << endl;
//    }

    max_x = corner_pnts[3][0];
    max_y = corner_pnts[3][1];
    min_x = corner_pnts[0][0];
    min_y = corner_pnts[0][1];


    //btTransform tm_left = btTransform(btQuaternion( 0,    0.9877,    0.1564 ,   0), psb->m_nodes[min_x_ind].m_x+btVector3(0,0,2));
    //btTransform tm_right = btTransform(btQuaternion( 0,    0.9877,    0.1564 ,   0), psb->m_nodes[max_x_ind].m_x+btVector3(0,0,2));
    //left_grabber->motionState->setKinematicPos(tm_left);
    //right_grabber->motionState->setKinematicPos(tm_right);

    //psb->appendAnchor(min_x_ind,left_grabber->rigidBody.get());
    //psb->appendAnchor(max_x_ind,right_grabber->rigidBody.get());


    btTransform tm_left = btTransform(btQuaternion( 0,    0,    0 ,   1), corner_pnts[0] + btVector3(left_gripper->children[0]->halfExtents[0],left_gripper->children[0]->halfExtents[1],0));
    left_gripper->setWorldTransform(tm_left);
    //left_gripper->toggle();


    btTransform tm_right = btTransform(btQuaternion( 0,    0,    0 ,   1), corner_pnts[2] + btVector3(-right_gripper->children[0]->halfExtents[0],right_gripper->children[0]->halfExtents[1],0));
    right_gripper->setWorldTransform(tm_right);

    btTransform tm_fixed1 = btTransform(btQuaternion( 0,    0,    0 ,   1), corner_pnts[1] + btVector3(fixed_gripper1->children[0]->halfExtents[0],-fixed_gripper1->children[0]->halfExtents[1],0));
    fixed_gripper1->setWorldTransform(tm_fixed1);
    //left_gripper->toggle();


    btTransform tm_fixed2 = btTransform(btQuaternion( 0,    0,    0 ,   1), corner_pnts[3] + btVector3(-fixed_gripper2->children[0]->halfExtents[0],-fixed_gripper2->children[0]->halfExtents[1],0));
    fixed_gripper2->setWorldTransform(tm_fixed2);


    //mirror about centerline along y direction
    //centerline defined by 2 points
    float mid_x = (max_x + min_x)/2;

    point_reflector.reset(new PointReflector(mid_x, min_y, max_y));
    //find node that most closely matches reflection of point
    for(int i = 0; i < node_pos.size(); i++)
    {

        //if(node_pos[i][0] < mid_x) //look at points in left half
        if(node_pos[i][0] < mid_x && (abs(node_pos[i][0] - min_x) < 0.01 || abs(node_pos[i][1] - min_y) < 0.01 || abs(node_pos[i][1] - max_y) < 0.01))
        {
            //float reflected_x = node_pos[i][0] + 2*(mid_x - node_pos[i][0]);
            btVector3 new_vec = point_reflector->reflect(node_pos[i]);
            float closest_dist = 100000;
            int closest_ind = -1;
            for(int j = 0; j < node_pos.size(); j++)
            {
                float dist = (node_pos[j]-new_vec).length();//(node_pos[j][0]-reflected_x)*(node_pos[j][0]-reflected_x) + (node_pos[j][1]-node_pos[i][1])*(node_pos[j][1]-node_pos[i][1]);
                if(dist < closest_dist)
                {
                    closest_dist = dist;
                    closest_ind = j;
                }
            }
            node_mirror_map[i] = closest_ind;
        }
    }





    //plotting

    std::vector<btVector3> plotpoints;
    std::vector<btVector4> plotcols;
    plotpoints.push_back(btVector3(mid_x,min_y,node_pos[0][2]));
    plotpoints.push_back(btVector3(mid_x,max_y,node_pos[0][2]));
    plotcols.push_back(btVector4(1,0,0,1));

//    for( map<int,int>::iterator ii=node_mirror_map.begin(); ii!=node_mirror_map.end(); ++ii)
//    {

//        cout << (*ii).first << ": " << (*ii).second << endl;
//        float r = (float)rand()/(float)RAND_MAX;
//        if(r < 0.5)
//        {
//            plotpoints.push_back(node_pos[(*ii).first]);
//            plotpoints.push_back(node_pos[(*ii).second]);
//            plotcols.push_back(btVector4(0,1,0,1));
//        }

//    }

    PlotLines::Ptr lines;
    lines.reset(new PlotLines(2));
    lines->setPoints(plotpoints,plotcols);
    env->add(lines);


    plot_points.reset(new PlotPoints());
    env->add(plot_points);


    left_center_point.reset(new PlotPoints(10));

    btTransform left_tm = left_gripper->getWorldTransform();
    cout << left_tm.getOrigin()[0] << " " << left_tm.getOrigin()[1] << " " << left_tm.getOrigin()[2] << " " <<endl;
    cout << mid_x << " " << min_y << " " << node_pos[0][2] <<endl;
    std::vector<btVector3> poinsfsefts2;
    //points2.push_back(left_tm.getOrigin());
    //points2.push_back(left_tm.getOrigin());
    std::vector<btVector4> plotcols2;
    plotcols2.push_back(btVector4(1,0,0,1));
    //plotcols2.push_back(btVector4(1,0,0,1));

    poinsfsefts2.push_back(btVector3(mid_x,min_y,node_pos[0][2]));
    //poinsfsefts2[0] = left_tm.getOrigin();
    //plotcols.push_back(btVector4(1,0,0,1));

    std::vector<btVector3> plotpoints2;
    plotpoints2.push_back( left_tm.getOrigin());
    //plotpoints2.push_back(btVector3(mid_x,max_y,node_pos[0][2]));


    env->add(left_center_point);
    //left_center_point->setPoints(plotpoints2);


    left_axes.reset(new PlotAxes());
    left_axes->setup(left_tm,1);
    env->add(left_axes);
//    drag_line.reset(new PlotLines(2));
//    env->add(drag_line);

    left_gripper->toggle();
    right_gripper->toggle();

    left_gripper->toggleattach(clothptr->softBody.get());
    right_gripper->toggleattach(clothptr->softBody.get());

    fixed_gripper1->toggle();
    fixed_gripper2->toggle();

    fixed_gripper1->toggleattach(clothptr->softBody.get());
    fixed_gripper2->toggleattach(clothptr->softBody.get());

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
