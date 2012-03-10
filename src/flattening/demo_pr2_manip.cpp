#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "robots/pr2.h"
#include <cstdlib>

#include "make_bodies.h"
#include "clothutil.h"
#include "nodeactions.h"
#include "clothgrasping.h"

class RobotInterpAction : public Action {
    RaveRobotKinematicObject::Ptr robot;
    vector<int> indices;
    vector<dReal> startvals, endvals;

public:
    RobotInterpAction(RaveRobotKinematicObject::Ptr robot_) : robot(robot_) { }

    void setIndices(const vector<int> &v) {
        indices.assign(v.begin(), v.end());
    }
    void setStartVals(const vector<dReal> &v) {
        startvals.assign(v.begin(), v.end());
    }
    void setEndVals(const vector<dReal> &v) {
        endvals.assign(v.begin(), v.end());
    }

    void step(float dt) {
        if (done()) return;
        stepTime(dt);
        const float a = fracElapsed();
        // interpolate each joint value
        vector<dReal> interpvals(startvals.size());
        for (int i = 0; i < interpvals.size(); ++i)
            interpvals[i] = (1.-a)*startvals[i] + a*endvals[i];
        robot->setDOFValues(indices, interpvals);
    }
};

class CustomScene : Scene {
    void runGripperAction(PR2SoftBodyGripperAction &a) {
        a.reset();
        a.toggleAction();
        runAction(a, BulletConfig::dt);
    }

public:
    void run() {
        const float table_height = .5;
        const float table_thickness = .05;
        BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale * btVector3(.75,.75,table_thickness/2),
            btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(0.8, 0, table_height-table_thickness/2))));
        table->rigidBody->setFriction(0.1);
        env->add(table);

        //const int resx = (int)(45*1.5), resy = (int)(31*1.5);
        const int resx = 45, resy = 31;
        const btScalar lenx = GeneralConfig::scale * 0.7, leny = GeneralConfig::scale * 0.5;
        const btVector3 clothcenter = GeneralConfig::scale * btVector3(0.5, 0, table_height+0.01);
        BulletSoftObject::Ptr cloth =
            makeSelfCollidingTowel(clothcenter, lenx, leny, resx, resy, env->bullet->softBodyWorldInfo);
        env->add(cloth);
        ClothSpec clothspec = { cloth->softBody.get(), resx, resy };

        PR2Manager pr2m(*this);

        PR2SoftBodyGripperAction leftAction(pr2m.pr2, pr2m.pr2Left->manip, true);
        leftAction.setTarget(cloth->softBody.get());
        leftAction.setExecTime(1.);
        addVoidKeyCallback('a', boost::bind(&CustomScene::runGripperAction, this, leftAction));

        startViewer();
        startFixedTimestepLoop(BulletConfig::dt);
    }
};

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = BulletConfig::internalTimeStep = 0.01;
    BulletConfig::maxSubSteps = 0;

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);

    CustomScene().run();

    return 0;
}
