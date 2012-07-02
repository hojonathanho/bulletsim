#ifndef COLAB_CLOTH_H
#define COLAB_CLOTH_H

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <omp.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <google/profiler.h>


//#define PROFILER
#define USE_PR2
#define DO_ROTATION
//#define USE_QUATERNION //NOT IMPLEMENTED!!!
//#define USE_TABLE
#define USE_RADIUS_CONTACT


#ifdef USE_PR2
#include <openrave/kinbody.h>
#include "robots/pr2.h"
#endif

//WARNING: THIS IS THE WRONG TRANSFORM, WILL NOT WORK FOR ROTATION!
const btTransform TBullet_PR2GripperRight = btTransform(btQuaternion(btVector3(0,1,0),3.14159265/2),btVector3(0,0,0))*btTransform(btQuaternion(btVector3(0,0,1),3.14159265/2),btVector3(0,0,0))*btTransform(btQuaternion(btVector3(0,1,0),3.14159265/2),btVector3(0,0,0));
//const btTransform TBullet_PR2GripperRight = btTransform(btQuaternion(btVector3(0,0,1),3.14159265/2),btVector3(0,0,0));
const btTransform TBullet_PR2GripperLeft = btTransform(btQuaternion(btVector3(0,1,0),3.14159265/2),btVector3(0,0,0))*btTransform(btQuaternion(btVector3(0,0,1),-3.14159265/2),btVector3(0,0,0))*btTransform(btQuaternion(btVector3(0,1,0),3.14159265/2),btVector3(0,0,0));

enum GripperState { GripperState_DONE, GripperState_CLOSING, GripperState_OPENING };

class GripperKinematicObject : public CompoundObject<BoxObject>{
public:

    float apperture;
    btTransform cur_tm;
    bool bOpen;
    bool bAttached;
    btVector3 halfextents;
    std::vector<int> vattached_node_inds;
    double closed_gap;
    GripperState state;

    typedef boost::shared_ptr<GripperKinematicObject> Ptr;

    GripperKinematicObject(btVector4 color = btVector4(0,0,1,0.3));
    void translate(btVector3 transvec);
    void applyTransform(btTransform tm);
    void setWorldTransform(btTransform tm);
    btTransform getWorldTransform(){return cur_tm;}
    void getWorldTransform(btTransform& in){in = cur_tm;}
    void toggle();
    void toggleattach(btSoftBody * psb, double radius = 0);
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
        o->state = state;
        o->closed_gap = closed_gap;
        o->vattached_node_inds = vattached_node_inds;
        o->halfextents = halfextents;
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



    void step_openclose(btSoftBody * psb) {
        if (state == GripperState_DONE) return;

        if(state == GripperState_OPENING && bAttached)
            toggleattach(psb);


        btTransform top_tm;
        btTransform bottom_tm;
        children[0]->motionState->getWorldTransform(top_tm);
        children[1]->motionState->getWorldTransform(bottom_tm);

        double step_size = 0.005;
        if(state == GripperState_CLOSING)
        {
            top_tm.setOrigin(top_tm.getOrigin() + step_size*top_tm.getBasis().getColumn(2));
            bottom_tm.setOrigin(bottom_tm.getOrigin() - step_size*bottom_tm.getBasis().getColumn(2));
        }
        else if(state == GripperState_OPENING)
        {
            top_tm.setOrigin(top_tm.getOrigin() - step_size*top_tm.getBasis().getColumn(2));
            bottom_tm.setOrigin(bottom_tm.getOrigin() + step_size*bottom_tm.getBasis().getColumn(2));
        }

        children[0]->motionState->setKinematicPos(top_tm);
        children[1]->motionState->setKinematicPos(bottom_tm);

//        if(state == GripperState_CLOSING && !bAttached)
//            toggleattach(psb, 0.5);

        double cur_gap_length = (top_tm.getOrigin() - bottom_tm.getOrigin()).length();
        if(state == GripperState_CLOSING && cur_gap_length <= (closed_gap + 2*halfextents[2]))
        {
            state = GripperState_DONE;
            bOpen = false;
            if(!bAttached)
                toggleattach(psb);

        }
        if(state == GripperState_OPENING && cur_gap_length >= apperture)
        {
            state = GripperState_DONE;
            bOpen = true;

        }

//        float frac = fracElapsed();
//        vals[0] = (1.f - frac)*startVal + frac*endVal;
//        manip->robot->setDOFValues(indices, vals);

//        if (vals[0] == CLOSED_VAL) {
//            attach(true);
//            attach(false);
//        }
    }


};

#ifdef USE_PR2
// I've only tested this on the PR2 model
class PR2SoftBodyGripperAction : public Action {
    RaveRobotObject::Manipulator::Ptr manip;
    dReal startVal, endVal;
    vector<int> indices;
    vector<dReal> vals;

    // min/max gripper dof vals
    //static const float CLOSED_VAL = 0.03f, OPEN_VAL = 0.54f;
    static const float CLOSED_VAL = 0.08f, OPEN_VAL = 0.54f;

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

        vals[0] = CLOSED_VAL;
        manip->robot->setDOFValues(indices, vals);
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
        //releaseAllAnchors();
    }

    void step(float dt) {
        if (done()) return;
        stepTime(dt);

        float frac = fracElapsed();
        vals[0] = (1.f - frac)*startVal + frac*endVal;
        manip->robot->setDOFValues(indices, vals);

//        if (vals[0] == CLOSED_VAL) {
//            attach(true);
//            attach(false);
//        }
    }
};
#endif

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

class StepState {
public:
    BulletInstance::Ptr bullet;
    OSGInstance::Ptr osg;
    Fork::Ptr fork;
    BulletSoftObject::Ptr cloth;
    GripperKinematicObject::Ptr left_gripper1;
    GripperKinematicObject::Ptr left_gripper2;
};




class CustomScene : public Scene {
public:
#ifdef USE_PR2
    PR2SoftBodyGripperAction::Ptr leftAction, rightAction;
    PR2Manager pr2m;
#endif

    GripperKinematicObject::Ptr left_gripper1, right_gripper1, left_gripper1_orig, right_gripper1_orig, left_gripper1_fork, right_gripper1_fork;
    GripperKinematicObject::Ptr left_gripper2, right_gripper2;
    struct {
        bool transGrabber0,rotateGrabber0,transGrabber1,rotateGrabber1, transGrabber2,rotateGrabber2, transGrabber3,rotateGrabber3, startDragging;
        float dx, dy, lastX, lastY;
    } inputState;

    int num_auto_grippers;
    bool bTracking, bInTrackingLoop;
    PointReflector::Ptr point_reflector;
    BulletSoftObject::Ptr clothptr, clothptr_orig, clothptr_fork;
    BulletInstance::Ptr bullet2;
    OSGInstance::Ptr osg2;
    Fork::Ptr fork;
    RaveRobotObject::Ptr origRobot, tmpRobot;
    std::map<int, int> node_mirror_map;
    std::vector<std::vector<double> > gripper_node_distance_map;
    float jacobian_sim_time;
    std::vector<btVector3> prev_node_pos;
    PlotPoints::Ptr plot_points;
    PlotPoints::Ptr left_center_point;
    PlotAxes::Ptr left_axes1,left_axes2;
    PlotLines::Ptr rot_lines;
    Eigen::MatrixXf cloth_distance_matrix;
    int user_mid_point_ind, robot_mid_point_ind;
    //std::vector<int> cloth_boundary_inds;


#ifdef USE_PR2
        CustomScene() : pr2m(*this){
#else
        CustomScene(){
#endif
        bTracking = bInTrackingLoop = false;
        inputState.transGrabber0 =  inputState.rotateGrabber0 =
                inputState.transGrabber1 =  inputState.rotateGrabber1 =
                inputState.transGrabber2 =  inputState.rotateGrabber2 =
                inputState.transGrabber3 =  inputState.rotateGrabber3 =
                inputState.startDragging = false;

        jacobian_sim_time = 0.05;

        left_gripper1_orig.reset(new GripperKinematicObject());
        left_gripper1_orig->setWorldTransform(btTransform(btQuaternion(0,0,0,1), btVector3(0,-10,0)));
        env->add(left_gripper1_orig);

        btVector4 color(0.6,0.6,0.6,1);//(1,0,0,0.0);
        right_gripper1_orig.reset(new GripperKinematicObject(color));
        right_gripper1_orig->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,10,0)));
        env->add(right_gripper1_orig);

        left_gripper1 = left_gripper1_orig;
        right_gripper1 = right_gripper1_orig;


        left_gripper2.reset(new GripperKinematicObject());
        left_gripper2->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,20,0)));
        env->add(left_gripper2);

        right_gripper2.reset(new GripperKinematicObject(color));
        right_gripper2->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0,-20,0)));
        env->add(right_gripper2);

        num_auto_grippers = 2;

        fork.reset();
    }


    BulletSoftObject::Ptr createCloth(btScalar s, const btVector3 &center);
    void createFork();
    void destroyFork();
    void swapFork();
    Eigen::MatrixXf computeJacobian();
    Eigen::MatrixXf computeJacobian_parallel();
    Eigen::MatrixXf computeJacobian_approx();
    double getDistfromNodeToClosestAttachedNodeInGripper(GripperKinematicObject::Ptr gripper, int input_ind, int &closest_ind);
    void simulateInNewFork(StepState& innerstate, float sim_time, btTransform& left_gripper1_tm, btTransform& left_gripper2_tm);
    void doJTracking();
    void drawAxes();
    void regraspWithOneGripper(GripperKinematicObject::Ptr gripper_to_attach, GripperKinematicObject::Ptr  gripper_to_detach);

    void run();
};

#endif // COLAB_CLOTH_H
