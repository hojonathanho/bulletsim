#include "clothmanip.h"

#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "nodeactions.h"

void liftClothEdges(Scene &scene, ClothSpec &cs, bool disableDrawing) {
    bool d = scene.drawingOn;
    scene.setDrawing(!disableDrawing);

    const int steps = 20;
//    const int steps = 50;
    const btVector3 force(0, 0, 0.5);
    for (int i = 0; i < steps; ++i) {
        for (int y = 0; y < cs.resy; ++y) {
            cs.psb->addForce(force, y*cs.resx + 0);
            cs.psb->addForce(force, y*cs.resx + cs.resx-1);
        }
        scene.step(BulletConfig::dt);
    }

    // let the cloth stabilize
    const int restingsteps = 400;
    for (int i = 0; i < restingsteps; ++i) {
        scene.step(BulletConfig::dt);
    }
    //scene.stepFor(BulletConfig::dt, 10);

    // clear velocities
    for (int i = 0; i < cs.psb->m_nodes.size(); ++i) {
        cs.psb->m_nodes[i].m_v.setZero();
    }

    scene.setDrawing(d);
}

void liftClothMiddle(Scene &scene, ClothSpec &cs, bool disableDrawing) {
    bool d = scene.drawingOn;
    scene.setDrawing(!disableDrawing);

    // add forces
    const int steps = 30;
//    const int steps = 50;
    const btVector3 force(0, 0, 0.5);
    for (int i = 0; i < steps; ++i) {
        for (int x = 0; x < cs.resx; ++x)
            cs.psb->addForce(force, (cs.resy / 2)*cs.resx + x);
        scene.step(BulletConfig::dt);
    }

    // let the cloth stabilize
    const int restingsteps = 400;
    for (int i = 0; i < restingsteps; ++i) {
        scene.step(BulletConfig::dt);
    }

    // clear velocities
    for (int i = 0; i < cs.psb->m_nodes.size(); ++i) {
        cs.psb->m_nodes[i].m_v.setZero();
    }
    scene.setDrawing(d);

    //liftClothMiddle2(scene, cs, disableDrawing);
}

// runs some random actions on a cloth
void randomizeCloth(Scene &scene, btSoftBody *psb, int resx, int resy, int numSteps) {
    NodeActionList actions;
    genSpecsForCloth(actions, resx, resy);
    for (int i = 0; i < numSteps; ++i) {
        NodeMoveAction &a = actions[rand() % actions.size()];
        a.setSoftBody(scene.env, psb);
        scene.runAction(a, BulletConfig::dt);
    }
}
