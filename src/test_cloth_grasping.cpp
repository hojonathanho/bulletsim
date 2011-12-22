#include "simplescene.h"
#include "softbodies.h"
#include "userconfig.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

const float clothMargin = 0.2;

class GripperAction : public Action {
    RaveRobotKinematicObject::Ptr robot;
    const float time;
    float timeElapsed;
    dReal startVal, endVal;
    vector<int> indices;
    vector<dReal> vals;

public:
    GripperAction(RaveRobotKinematicObject::Ptr robot_, const string &jointName, float time_) :
            robot(robot_),
            time(time_), timeElapsed(0.f) {
        int idx = robot->robot->GetJointIndex(jointName);
        indices.push_back(idx);
        vals.push_back(0);
    }
    void setEndpoints(dReal start, dReal end) { startVal = start; endVal = end; }

    void step(float dt) {
        if (timeElapsed >= time) {
            setDone(true);
            return;
        }
        timeElapsed += dt;

        float frac = timeElapsed / time;
        vals[0] = (1.f - frac)*startVal + frac*endVal;
        robot->setDOFValues(indices, vals);
    }
};

BulletSoftObject::Ptr createCloth(Scene &scene, btScalar s, const btVector3 &center, int divs) {
    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        scene.env->bullet->softBodyWorldInfo,
        center + btVector3(-s,-s,0),
        center + btVector3(+s,-s,0),
        center + btVector3(-s,+s,0),
        center + btVector3(+s,+s,0),
        divs, divs,
        0, true);

    psb->m_cfg.piterations = 8;
    psb->getCollisionShape()->setMargin(clothMargin);
    btSoftBody::Material* pm=psb->appendMaterial();
    pm->m_kLST		=	0.1;
    psb->generateBendingConstraints(2, pm);
    psb->setTotalMass(150);
    //psb->generateClusters(1024);
    //psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS;// + btSoftBody::fCollision::CL_SELF;

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

int main(int argc, char *argv[]) {
    Config::read(argc, argv);
    CFG.scene.scale = 20.;

    const float table_height = .5;
    const float table_thickness = .05;
    boost::shared_ptr<btDefaultMotionState> ms(new btDefaultMotionState(
        btTransform(btQuaternion(0, 0, 0, 1),
                    CFG.scene.scale * btVector3(1, 0, table_height-table_thickness/2))));
    boost::shared_ptr<BulletObject> table(
        new BoxObject(0, CFG.scene.scale * btVector3(.75,.75,table_thickness/2),ms));

    Scene s;
    s.env->add(table);
    s.env->add(createCloth(s, CFG.scene.scale * 0.25, CFG.scene.scale * btVector3(0.75, 0, 1.5), 40));

    s.startViewer();
    //s.setSyncTime(true);

    // open gripper
    const float dt = 0.01f;
    //CFG.scene.mouseDragScale = 0.1;
    CFG.bullet.internalTimeStep = dt;
    CFG.bullet.maxSubSteps = 0;
    s.stepFor(dt, 2);

    GripperAction leftAction(s.pr2, "l_gripper_l_finger_joint", 1);
    leftAction.setEndpoints(0, .54); // .54 is the max joint value for the pr2
    s.runAction(leftAction, dt);

    GripperAction rightAction(s.pr2, "r_gripper_l_finger_joint", 1);
    rightAction.setEndpoints(0, .54);
    s.runAction(rightAction, dt);

    s.startLoop();
    //s.startFixedTimestepLoop(dt);
    return 0;
}
