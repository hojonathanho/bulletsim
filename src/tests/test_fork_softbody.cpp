#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"

#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>

struct CustomScene : public Scene {
    BoxObject::Ptr table;
    BulletSoftObject::Ptr cloth;
    SphereObject::Ptr sphere;

    BulletSoftObject::Ptr createCloth(btScalar s, const btVector3 &center, int divs);

    void run();
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
        case ' ':
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

    psb->m_cfg.piterations = 2;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
        | btSoftBody::fCollision::CL_RS;
//        | btSoftBody::fCollision::CL_SELF;
    psb->getCollisionShape()->setMargin(0.4);
    btSoftBody::Material *pm = psb->appendMaterial();
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    psb->setTotalMass(1, true);
    psb->generateClusters(512);

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

void CustomScene::run() {
    viewer.addEventHandler(new CustomKeyHandler(*this));

    const float dt = BulletConfig::dt;
    const float table_height = .5;
    const float table_thickness = .05;
    boost::shared_ptr<btDefaultMotionState> ms(new btDefaultMotionState(
        btTransform(btQuaternion(0, 0, 0, 1),
                    GeneralConfig::scale * btVector3(1.25, 0, table_height-table_thickness/2))));
    table.reset(new BoxObject(0, GeneralConfig::scale * btVector3(.75,.75,table_thickness/2),ms));
    env->add(table);

    cloth = createCloth(GeneralConfig::scale * 0.25, GeneralConfig::scale * btVector3(1, 0, 1), 31);
    env->add(cloth);

    const float radius = 0.1;
    ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1),
                    GeneralConfig::scale * btVector3(1.1, 0, table_height + radius))));
    sphere.reset(new SphereObject(1, GeneralConfig::scale * radius, ms));
    env->add(sphere);

    startViewer();

    stepFor(dt, 0.5);

    BulletInstance::Ptr bullet2(new BulletInstance);
    bullet2->setGravity(BulletConfig::gravity);
    OSGInstance::Ptr osg2(new OSGInstance);
    osg->root->addChild(osg2->root.get());
    Fork::Ptr fork(new Fork(env, bullet2, osg2));
    registerFork(fork);
    cout << "environment copied" << endl;

    SphereObject::Ptr sphere2 = boost::static_pointer_cast<SphereObject> (fork->forkOf(sphere));
    // apply force in second world on sphere for 0.5 secs
    for (int i = 0; i < 0.5/dt; ++i) {
        sphere2->rigidBody->applyCentralForce(GeneralConfig::scale * btVector3(-30, -30, 0));
        fork->env->step(dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
        step(dt);
        idleFor(dt);
    }

    startFixedTimestepLoop(dt);
}

int main(int argc, char *argv[]) {
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = BulletConfig::internalTimeStep = 0.002;
    BulletConfig::maxSubSteps = 0;
    SceneConfig::enableIK = false;
    SceneConfig::enableHaptics = false;
    SceneConfig::enableRobot = false;
    GeneralConfig::scale = 20;
    BulletConfig::gravity *= GeneralConfig::scale;

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);

    CustomScene().run();
    return 0;
}
