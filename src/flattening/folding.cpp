#include "folding.h"

#include "cloth.h"
#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "simulation/logging.h"

int FoldingConfig::foldHalfCircleDivs = 10;
int FoldingConfig::attractNodesMaxSteps = 50;
float FoldingConfig::attractNodesMaxAvgErr = 0.1;
bool FoldingConfig::disableGravityWhenFolding = true;

float FoldingConfig::dropHeight = 0.2;
float FoldingConfig::dropWaitTime = 4;
float FoldingConfig::dropTimestep = 0.03;

static void attractNodesToPositions(Scene &scene, Cloth &c, const vector<btVector3> &pos) {
    btSoftBody *psb = c.psb();
    BOOST_ASSERT(pos.size() == psb->m_nodes.size());
    const btScalar dt = BulletConfig::dt;
    const btScalar nodemass = psb->getTotalMass() / psb->m_nodes.size();
    const int MAXSTEPS = FoldingConfig::attractNodesMaxSteps;
    const btScalar MAXAVGERR = FoldingConfig::attractNodesMaxAvgErr; // average error of node position
    btScalar avgerr = SIMD_INFINITY;
    int step;
    for (step = 0; step < MAXSTEPS && avgerr > MAXAVGERR; ++step) {
        for (int i = 0; i < pos.size(); ++i) {
            btVector3 dir = pos[i] - psb->m_nodes[i].m_x;
            if (btFuzzyZero(dir.length())) continue;
//            dir.normalize();
            btVector3 force = 2*nodemass/dt/dt/250 * dir;
            psb->addForce(force, i);
        }
        scene.step(dt, 0, dt);

        avgerr = 0;
        for (int i = 0; i < pos.size(); ++i)
            avgerr += (pos[i] - psb->m_nodes[i].m_x).length();
        avgerr /= pos.size();
    }
    if (step >= MAXSTEPS)
        LOG_DEBUG("hit step limit");
}

struct Rotspec {
    int idx;
    btVector3 radius;
    btVector3 centerpt;
};

void Folding::foldClothAlongLine(Scene &scene, Cloth &c, const btVector3 &a, const btVector3 &b) {
    if (FoldingConfig::disableGravityWhenFolding)
        scene.bullet->setGravity(btVector3(0, 0, 0));

    btVector3 line = b - a; line.setZ(0); line.normalize();
    btSoftBody *psb = c.psb();

    // for each node to the left of the line, apply forces to
    // make it go in a half-circle trajectory to the other side
    vector<Rotspec> toFold; // maps points to traj radius
    for (int i = 0; i < psb->m_nodes.size(); ++i) {
        const btVector3 &x = psb->m_nodes[i].m_x;
        btVector3 ptline = x - a; ptline.setZ(0);
        if (line.cross(ptline).z() <= 0) continue;

        btVector3 proj = (ptline.dot(line)/line.dot(line))*line;
        btVector3 radius = ptline - proj;
        Rotspec spec = { i, radius, proj + a };
        toFold.push_back(spec);
    }

    vector<btVector3> pos; // the "desired" node positions
    pos.resize(psb->m_nodes.size());
    for (int i = 0; i < psb->m_nodes.size(); ++i)
        pos[i] = psb->m_nodes[i].m_x;

    for (int s = 0; s <= FoldingConfig::foldHalfCircleDivs; ++s) {
        btScalar angle = M_PI*s/FoldingConfig::foldHalfCircleDivs;
        for (int z = 0; z < toFold.size(); ++z)
            pos[toFold[z].idx] = toFold[z].radius.rotate(line, angle) + toFold[z].centerpt;
        attractNodesToPositions(scene, c, pos);
    }

    if (FoldingConfig::disableGravityWhenFolding)
        scene.bullet->setGravity(BulletConfig::gravity);
}

void Folding::pickUpAndDrop(Scene &scene, Cloth &cloth) {
    // temporarily decrease damping
    btScalar oldkDF = cloth.psb()->m_cfg.kDF;
    cloth.psb()->m_cfg.kDF = 0;

    // generate random rotation
    btScalar y  = (btScalar) rand() / RAND_MAX * 2 * M_PI;
    btScalar p  = (btScalar) rand() / RAND_MAX * 2 * M_PI;
    btScalar r  = (btScalar) rand() / RAND_MAX * 2 * M_PI;
    cloth.translateRelToCenter(btTransform(btQuaternion(y, p, r),
                btVector3(0, 0, FoldingConfig::dropHeight*METERS)));

    scene.stepFor(FoldingConfig::dropTimestep, FoldingConfig::dropWaitTime);

    cloth.psb()->m_cfg.kDF = oldkDF;
    LOG_DEBUG("done dropping");
}
