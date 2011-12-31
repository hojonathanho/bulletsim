#include "simplescene.h"
#include "softbodies.h"
#include "userconfig.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <openrave/kinbody.h>

// I've only tested this on the PR2 model
class GripperAction : public Action {
    RaveRobotKinematicObject::Manipulator::Ptr manip;
    dReal startVal, endVal;
    vector<int> indices;
    vector<dReal> vals;

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

public:
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

    btVector3 getInnerPt(bool left) const {
        // first get some innermost point on the gripper
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

    typedef boost::shared_ptr<GripperAction> Ptr;
    GripperAction(RaveRobotKinematicObject::Manipulator::Ptr manip_,
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
    }

    void setEndpoints(dReal start, dReal end) { startVal = start; endVal = end; }
    dReal getCurrDOFVal() const {
        vector<dReal> v;
        manip->robot->robot->GetDOFValues(v);
        return v[indices[0]];
    }
    void setOpenAction() { setEndpoints(getCurrDOFVal(), 0.54f); } // 0.54 is the max joint value for the pr2
    void setCloseAction() { setEndpoints(getCurrDOFVal(), 0.f); }

    void step(float dt) {
        if (done()) return;
        stepTime(dt);

        float frac = fracElapsed();
        vals[0] = (1.f - frac)*startVal + frac*endVal;
        manip->robot->setDOFValues(indices, vals);
    }
};


struct CustomScene : public Scene {
    GripperAction::Ptr leftAction, rightAction;
    BulletSoftObject::Ptr createCloth(btScalar s, const btVector3 &center, int divs);
    void run();
    void printDiagnostics();
};

class CustomKeyHandler : public osgGA::GUIEventHandler {
    CustomScene &scene;
public:
    CustomKeyHandler(CustomScene &scene_) : scene(scene_) { }
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
};

bool CustomKeyHandler::handle(const osgGA::GUIEventAdapter &ea,osgGA::GUIActionAdapter &) {
    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case 'a':
            scene.leftAction->reset();
            scene.leftAction->setCloseAction();
            scene.runAction(scene.leftAction, CFG.bullet.dt);
            break;
        case 's':
            scene.rightAction->reset();
            scene.rightAction->setCloseAction();
            scene.runAction(scene.rightAction, CFG.bullet.dt);
            break;

        case ' ':
            scene.printDiagnostics();
            break;
        }
        break;
    }
    return false;
}

BulletSoftObject::Ptr CustomScene::createCloth(btScalar s, const btVector3 &center, int divs) {
    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        env->bullet->softBodyWorldInfo,
        center + btVector3(-s,-s,0),
        center + btVector3(+s,-s,0),
        center + btVector3(-s,+s,0),
        center + btVector3(+s,+s,0),
        divs, divs,
        0, true);

    psb->m_cfg.piterations = 4;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
//        | btSoftBody::fCollision::SDF_RS // so we can use m_rcontacts
        | btSoftBody::fCollision::CL_RS
        | btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 0.9;
    psb->getCollisionShape()->setMargin(0.04);
    btSoftBody::Material *pm = psb->appendMaterial();
//    pm->m_kLST = 0.4;
//    pm->m_kAST = 0.4;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    psb->setTotalMass(100, true);
    psb->generateClusters(0);

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

void CustomScene::printDiagnostics() {
#if 0
    struct S {
        static void directionInfo(RaveRobotKinematicObject::Manipulator::Ptr p) {
            printf("name: %s\n", p->manip->GetName().c_str());
            btTransform manipRot(util::toBtTransform(p->manip->GetTransform()));
            manipRot.setOrigin(btVector3(0, 0, 0));

            btVector3 closingNormal = manipRot
                * btVector3(p->manip->GetClosingDirection()[0],
                            p->manip->GetClosingDirection()[1],
                            p->manip->GetClosingDirection()[2]);
            btVector3 tv = closingNormal;
            printf("world closing normal: %f %f %f\n", tv.x(), tv.y(), tv.z());

            btVector3 toolDirection = manipRot * util::toBtVector(p->manip->GetLocalToolDirection());
            tv = toolDirection;
            printf("world tool direction: %f %f %f\n", tv.x(), tv.y(), tv.z());

            btVector3 closingDirection = toolDirection.cross(closingNormal);
            tv = closingDirection;
            printf("world closing direction (left finger): %f %f %f\n", tv.x(), tv.y(), tv.z());
            tv = -closingDirection;
            printf("world closing direction (right finger): %f %f %f\n", tv.x(), tv.y(), tv.z());
        }
    };
#endif

    btVector3 tv = leftAction->getClosingDirection(true);
    printf("world closing direction (left finger): %f %f %f\n", tv.x(), tv.y(), tv.z());
    tv = leftAction->getClosingDirection(false);
    printf("world closing direction (right finger): %f %f %f\n", tv.x(), tv.y(), tv.z());
    printf("\n");

    tv = rightAction->getClosingDirection(true);
    printf("world closing direction (left finger): %f %f %f\n", tv.x(), tv.y(), tv.z());
    tv = rightAction->getClosingDirection(false);
    printf("world closing direction (right finger): %f %f %f\n", tv.x(), tv.y(), tv.z());
    printf("\n");
}

void CustomScene::run() {
    viewer.addEventHandler(new CustomKeyHandler(*this));

    const float dt = CFG.bullet.dt;
    const float table_height = .5;
    const float table_thickness = .05;
    boost::shared_ptr<btDefaultMotionState> ms(new btDefaultMotionState(
        btTransform(btQuaternion(0, 0, 0, 1),
                    CFG.scene.scale * btVector3(1.25, 0, table_height-table_thickness/2))));
    BoxObject::Ptr table(
        new BoxObject(0, CFG.scene.scale * btVector3(.75,.75,table_thickness/2),ms));

    BulletSoftObject::Ptr cloth(
            createCloth(CFG.scene.scale * 0.25, CFG.scene.scale * btVector3(1, 0, 1), 31));

    env->add(table);
    env->add(cloth);

    leftAction.reset(new GripperAction(pr2Left, "l_gripper_l_finger_tip_link", "l_gripper_r_finger_tip_link", 1));
    rightAction.reset(new GripperAction(pr2Right, "r_gripper_l_finger_tip_link", "r_gripper_r_finger_tip_link", 1));

    //setSyncTime(true);
    startViewer();
    stepFor(dt, 2);

    leftAction->setOpenAction();
    runAction(leftAction, dt);

    rightAction->setOpenAction();
    runAction(rightAction, dt);

    //startLoop();
//    startFixedTimestepLoop(dt);




    vector<btVector3> tmpPts, tmpLines0, tmpLines1;
#if 0
    for (int i = 0; i < cloth->softBody->m_faces.size(); ++i) {
        for (int j = 0; j < 3; ++j)
            tmpPts.push_back(cloth->softBody->m_faces[i].m_n[j]->m_x);
    }
    plotPoints->setPoints(tmpPts);
#endif

    btCollisionObject *llfinger = pr2->associatedObj(pr2->robot->GetLink("l_gripper_l_finger_tip_link"))->rigidBody.get();
    btSoftBody * const psb = cloth->softBody.get();
    while (true) {

        // custom contact checking
        // adapted from btSoftBody.cpp and btSoftBodyInternals.h
        struct  Custom_CollideSDF_RS : btDbvt::ICollide
        {
                void            Process(const btDbvtNode* leaf)
                {
                        btSoftBody::Node*       node=(btSoftBody::Node*)leaf->data;
                        DoNode(*node);
                }
                void            DoNode(btSoftBody::Node& n)
                {
                        const btScalar                  m=n.m_im>0?dynmargin:stamargin;
                        btSoftBody::RContact    c;
                        if(     (!n.m_battach)&&
                                psb->checkContact(m_colObj1,n.m_x,m,c.m_cti))
                        {
                                const btScalar  ima=n.m_im;
                                const btScalar  imb= m_rigidBody? m_rigidBody->getInvMass() : 0.f;
                                const btScalar  ms=ima+imb;
                                if(ms>0)
                                {
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
                btSoftBody::tRContactArray rcontacts;
        };


        btCollisionObject *pco = llfinger;


        Custom_CollideSDF_RS  docollide;              
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

        tmpPts.clear();
        tmpLines0.clear(); tmpLines1.clear();
        tmpLines0.push_back(leftAction->getInnerPt(true));
        tmpLines1.push_back(leftAction->getInnerPt(true) + CFG.scene.scale * (leftAction->getManipRot() * leftAction->getClosingDirection(true)));

        btVector3 v = leftAction->getInnerPt(true);
        cout << "inner point: " << v.x() << ' ' << v.y() << ' ' << v.z() << '\n';
        v = pr2Left->getTransform().getOrigin();
        cout << "center point: " << v.x() << ' ' << v.y() << ' ' << v.z() << '\n';


        for (int i = 0; i < docollide.rcontacts.size(); ++i) {
            const btSoftBody::RContact &c = docollide.rcontacts[i];
            KinBody::LinkPtr colLink = pr2->associatedObj(c.m_cti.m_colObj);
            if (!colLink) continue;
            cout << "robot contact: " << colLink->GetName() << '\n';

            //const btVector3 contactPt = c.m_node->m_x - c.m_cti.m_normal*(btDot(c.m_node->m_x, c.m_cti.m_normal) + c.m_cti.m_offset);
            const btVector3 &contactPt = c.m_node->m_x;
            cout << "contact point: " << contactPt.x() << ' ' << contactPt.y() << ' ' << contactPt.z() << '\n';

            if (leftAction->onInnerSide(contactPt, true)) {
                cout << "\ton inner side\n";
                tmpPts.push_back(contactPt);
            }
#if 0
            if (colLink->GetName() == "l_gripper_l_finger_tip_link") {
                cout << "touching l_gripper_l_finger_tip_link" << endl;
            }
#endif
        }
        plotPoints->setPoints(tmpPts);
        plotLines->setPoints(tmpLines0, tmpLines1);

        step(dt);
    }
}

int main(int argc, char *argv[]) {
    Config::read(argc, argv);
    //CFG.scene.mouseDragScale = 0.1;
    CFG.scene.scale = 20.;
    CFG.viewer.cameraHomePosition = btVector3(100, 0, 100);
    CFG.bullet.dt = 0.01;
    CFG.bullet.internalTimeStep = 0.01;
    CFG.bullet.maxSubSteps = 0;

    CustomScene().run();
    return 0;
}
