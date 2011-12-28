#include "simplescene.h"
#include "softbodies.h"
#include "userconfig.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

// I've only tested this on the PR2 model
class GripperAction : public Action {
    RaveRobotKinematicObject::Manipulator::Ptr manip;
    dReal startVal, endVal;
    vector<int> indices;
    vector<dReal> vals;

    // vector normal to the direction that the gripper fingers move in the manipulator frame
    // (on the PR2 this points back into the arm)
    const btVector3 closingNormal;

    // points straight down in the PR2 initial position (manipulator frame)
    const btVector3 toolDirection;

    btTransform getManipRot() const {
        btTransform trans(util::toBtTransform(manip->manip->GetTransform()));
        trans.setOrigin(btVector3(0, 0, 0));
        return trans;
    }

    // Returns the direction that the specified finger will move when closing
    // (manipulator frame)
    btVector3 getClosingDirection(bool left) const {
        return (left ? 1 : -1) * toolDirection.cross(closingNormal);
    }

    // Returns true is pt is on the inner side of the specified finger of the gripper
    bool onInnerSide(const btVector3 &pt, bool left) const {
        // first get some innermost point on the gripper
        btVector3 x;
        cout << "NOT IMPLEMENTED" << endl;
        BOOST_ASSERT(false);

        // then the point and the closing direction define the plane
        return (getManipRot() * getClosingDirection(left)).dot(pt - x) > 0;
    }

public:
    typedef boost::shared_ptr<GripperAction> Ptr;
    GripperAction(RaveRobotKinematicObject::Manipulator::Ptr manip_, float time) :
            Action(time), manip(manip_), vals(1, 0),
            indices(manip->manip->GetGripperIndices()),
            closingNormal(manip->manip->GetClosingDirection()[0],
                          manip->manip->GetClosingDirection()[1],
                          manip->manip->GetClosingDirection()[2]),
            toolDirection(util::toBtVector(manip->manip->GetLocalToolDirection()))
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
        + btSoftBody::fCollision::CL_RS
        + btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 0.9;
    psb->getCollisionShape()->setMargin(0.04);
    btSoftBody::Material *pm = psb->appendMaterial();
//    pm->m_kLST = 0.4;
//    pm->m_kAST = 0.4;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    psb->setTotalMass(1, true);
    psb->generateClusters(0);

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

#define PV(v) {for(int z=0;z<(v).size();++z)cout<<(v)[z]<<' ';cout<<'\n';}
void CustomScene::printDiagnostics() {
    printf("%d\n",pr2->robot->GetJointIndex("l_gripper_l_finger_joint"));
    PV(pr2Left->manip->GetGripperIndices());
    printf("\n");

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

            printf("\n");
        }
    };

    S::directionInfo(pr2Left);
    S::directionInfo(pr2Right);
}

void CustomScene::run() {
    viewer.addEventHandler(new CustomKeyHandler(*this));

    const float dt = CFG.bullet.dt;
    const float table_height = .5;
    const float table_thickness = .05;
    boost::shared_ptr<btDefaultMotionState> ms(new btDefaultMotionState(
        btTransform(btQuaternion(0, 0, 0, 1),
                    CFG.scene.scale * btVector3(1.25, 0, table_height-table_thickness/2))));
    boost::shared_ptr<BulletObject> table(
        new BoxObject(0, CFG.scene.scale * btVector3(.75,.75,table_thickness/2),ms));

    env->add(table);
    env->add(createCloth(CFG.scene.scale * 0.25, CFG.scene.scale * btVector3(1, 0, 1), 31));

    leftAction.reset(new GripperAction(pr2Left, 1));
    rightAction.reset(new GripperAction(pr2Right, 1));

    //setSyncTime(true);
    startViewer();
    stepFor(dt, 2);

    leftAction->setOpenAction();
    runAction(leftAction, dt);

    rightAction->setOpenAction();
    runAction(rightAction, dt);

    //startLoop();
    startFixedTimestepLoop(dt);
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
