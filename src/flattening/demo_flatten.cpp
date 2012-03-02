#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <cstdlib>

#include "clothmanip.h"
#include "nodeactions.h"
#include "search.h"

static BulletSoftObject::Ptr createCloth(Scene &scene, btScalar s, int resx, int resy, const btVector3 &center) {
    btSoftBody *psb = btSoftBodyHelpers::CreatePatch(
        scene.env->bullet->softBodyWorldInfo,
        center + btVector3(-s,-s,0),
        center + btVector3(+s,-s,0),
        center + btVector3(-s,+s,0),
        center + btVector3(+s,+s,0),
        resx, resy,
        0, true);

    psb->m_cfg.piterations = 2;
    psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS
        | btSoftBody::fCollision::CL_RS
        | btSoftBody::fCollision::CL_SELF;
    psb->m_cfg.kDF = 0.6;
    psb->m_cfg.kSSHR_CL = 1.0; // so the cloth doesn't penetrate itself
    psb->m_cfg.kSRHR_CL = 1.0;
    psb->m_cfg.kSKHR_CL = 1.0;

    psb->getCollisionShape()->setMargin(0.05);
    btSoftBody::Material *pm = psb->appendMaterial();
    pm->m_kLST = 1;
    pm->m_kAST = 1;
    psb->generateBendingConstraints(2, pm);
    psb->randomizeConstraints();
    psb->setTotalMass(1, true);
    psb->generateClusters(0);

    return BulletSoftObject::Ptr(new BulletSoftObject(psb));
}

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 20.;
    ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
    BulletConfig::dt = BulletConfig::internalTimeStep = 0.02;
    BulletConfig::maxSubSteps = 0;

    Parser parser;

    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);

    Scene scene;

    const float table_height = .5;
    const float table_thickness = .05;
    BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale * btVector3(.75,.75,table_thickness/2),
        btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(1.2, 0, table_height-table_thickness/2))));
    table->rigidBody->setFriction(0.5);
    scene.env->add(table);

    const int divs = 31;
    BulletSoftObject::Ptr cloth(
        createCloth(scene, GeneralConfig::scale * 0.25, divs, divs, GeneralConfig::scale * btVector3(0.9, 0, table_height+0.01)));
    scene.env->add(cloth);
    ClothSpec clothspec = { cloth->softBody.get(), divs, divs };

    NodeActionList actions;
    genSpecsForCloth(actions, clothspec.resx, clothspec.resy);

    scene.startViewer();
    scene.stepFor(BulletConfig::dt, 1);

    cout << "lifting..." << endl;
    liftClothMiddle(scene, clothspec);
    cout << "done lifting."<< endl;
    scene.idle(true);

    NodeActionList optimal;
//    scene.setDrawing(false);
    flattenCloth_greedy(scene, cloth, actions, 10, optimal);
    scene.setDrawing(true);
    scene.idle(true);

    // replay optimal actions on cloth
    for (int i = 0; i < optimal.size(); ++i) {
        NodeMoveAction &a = optimal[i];
        a.setSoftBody(scene.env, cloth->softBody.get());
        while (!a.done()) {
            a.step(BulletConfig::dt);
            scene.env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
            scene.draw();
        }
        scene.idle(true);
    }
    scene.idle(true);

    return 0;
}
