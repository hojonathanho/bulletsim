#include "simplescene.h"
#include "softbodies.h"
#include "userconfig.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

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
    void setVals(dReal start, dReal end) { startVal = start; endVal = end; }

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

void createCloth(Scene &scene, btScalar s, btScalar z, int divs) {
    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(scene.env->bullet->softBodyWorldInfo,
        btVector3(-s,-s,z),
        btVector3(+s,-s,z),
        btVector3(-s,+s,z),
        btVector3(+s,+s,z),
        divs, divs,
        0/*1+2+4+8*/, true);

    psb->m_cfg.piterations = 2;
    psb->getCollisionShape()->setMargin(0.2);
    btSoftBody::Material* pm=psb->appendMaterial();
    pm->m_kLST		=	0.4;
    psb->generateBendingConstraints(2, pm);
    psb->setTotalMass(150);
    //psb->createClotherateClusters(1024);
    //psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS;// + btSoftBody::fCollision::CL_SELF;

    scene.env->add(BulletSoftObject::Ptr(new BulletSoftObject(psb)));
}

int main(int argc, char *argv[]) {
    Config::read(argc, argv);
    CFG.scene.scale = 10.;
    Scene scene;

    //createCloth(scene, 5, 30, 40);

    scene.startViewer();

    // open gripper
    const float dt = 0.01f;
    scene.stepFor(dt, 2);

    GripperAction leftAction(scene.pr2, "l_gripper_l_finger_joint", 1);
    leftAction.setVals(0, .54);
    scene.runAction(leftAction, dt);

    GripperAction rightAction(scene.pr2, "r_gripper_l_finger_joint", 1);
    rightAction.setVals(0, .54);
    scene.runAction(rightAction, dt);

    scene.startLoop();
    return 0;
}
