#include "search.h"
#include <omp.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <Wm5Core.h>
#include <Wm5Mathematics.h>
#include <limits>
#include "simulation/config_bullet.h"
#include "simulation/simplescene.h"
using std::numeric_limits;

static btVector3 softBodyCM(btSoftBody *psb) {
    // assumes all node masses are the same..
    btVector3 sum(0, 0, 0);
    for (int i = 0; i < psb->m_nodes.size(); ++i)
        sum += psb->m_nodes[i].m_x;
    return sum * 1./psb->m_nodes.size();
}

// project the cloth onto the xy-plane, and find the area
// of the convex hull of the projected vertices
static btScalar projectedConvexHullArea(btSoftBody *psb) {
    Wm5::Vector2f *vertices = new Wm5::Vector2f[psb->m_nodes.size()];
    for (int i = 0; i < psb->m_nodes.size(); ++i) {
        vertices[i].X() = psb->m_nodes[i].m_x.x();
        vertices[i].Y() = psb->m_nodes[i].m_x.y();
    }

    Wm5::ConvexHullf *hull = new Wm5::ConvexHull2f(psb->m_nodes.size(), vertices, 0.001f, false, Wm5::Query::QT_REAL);
    const int nSimplices = hull->GetNumSimplices();
    const int *indices = hull->GetIndices();
    Wm5::Vector2f *hullvertices = new Wm5::Vector2f[nSimplices];
    for (int i = 0; i < nSimplices; ++i)
        hullvertices[i] = vertices[indices[i]];

    btScalar area = (btScalar) Wm5::Polygon2f(nSimplices, hullvertices).ComputeArea();

//    delete [] hullvertices;
    delete hull;
    delete [] vertices;

    return area;
}

struct StepState {
    BulletInstance::Ptr bullet;
    OSGInstance::Ptr osg;
    Fork::Ptr fork;
    BulletSoftObject::Ptr cloth;
};

static NodeMoveAction::Spec flattenCloth_greedy_single_internal(const StepState &initState, const vector<NodeMoveAction::Spec> &candspecs, StepState &newState, Scene *pscene=NULL, float color_r=0, float color_g=0, float color_b=0, float color_a=0) {
    boost::posix_time::ptime begTick(boost::posix_time::microsec_clock::local_time());

    // at each step, try all actions and find the one that flattens the cloth
    struct OptimalSpec {
        StepState state;
        NodeMoveAction::Spec actionspec;
        float val;
    };

    btVector3 startingCM = softBodyCM(initState.cloth->softBody.get());

    // pre-filter candspecs
    vector<NodeMoveAction::Spec> candspecs2;
    for (int i = 0; i < candspecs.size(); ++i) {
        // ignore actions that drag down the cloth (for now)
        if (candspecs[i].v.z() != 0) continue;

        // ignore actions that drag a node toward the center
        if (candspecs[i].v.dot(initState.cloth->softBody->m_nodes[candspecs[i].i].m_x - startingCM) < 0) continue;

        // ignore actions on nodes that aren't visible
//        if (!checkVisibility(
        candspecs2.push_back(candspecs[i]);
    }

    // do a manual reduction over the array
    // each thread t sets threadoptimal[t] to be the optimal in its share of work
    // then at the end we take the optimal of threadoptimal[t] over all t
    OptimalSpec *threadoptimal = NULL;
    int numthreads;

    #pragma omp parallel shared(threadoptimal, numthreads)
    {
        #pragma omp single
        {
            numthreads = omp_get_num_threads();
            threadoptimal = new OptimalSpec[numthreads];
            for (int i = 0; i < numthreads; ++i) {
                threadoptimal[i].val = numeric_limits<float>::infinity();
            }
        }

        NodeMoveAction ac;

        // try out each action in candspecs2 (filtered candspecs)
        #pragma omp for
        for (int i = 0; i < candspecs2.size(); ++i) {
            ac.reset();
            ac.spec = candspecs2[i];
            ac.readSpec();

            //cout << "\taction " << (i+1) << "/" << candspecs2.size() << '\n';

            // to try the current action, fork off the initState
            StepState innerstate;
            innerstate.bullet.reset(new BulletInstance);
            innerstate.bullet->setGravity(BulletConfig::gravity);
            innerstate.osg.reset(new OSGInstance);
            innerstate.fork.reset(new Fork(initState.fork->env, innerstate.bullet, innerstate.osg));
            innerstate.cloth = boost::static_pointer_cast<BulletSoftObject>(
                    innerstate.fork->forkOf(initState.cloth));

            // drawing if requested
            if (numthreads == 1 && pscene) {
                pscene->osg->root->addChild(innerstate.osg->root.get());
                innerstate.cloth->setColor(color_r, color_g, color_b, color_a);
            }

            // now run the action on the innerstate
            ac.setSoftBody(innerstate.fork->env, innerstate.cloth->softBody.get());
            while (!ac.done()) {
                ac.step(BulletConfig::dt);
                innerstate.fork->env->step(BulletConfig::dt,
                        BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);

                if (numthreads == 1 && pscene)
                    pscene->draw();
            }

            if (numthreads == 1 && pscene)
                pscene->osg->root->removeChild(innerstate.osg->root.get());

            float avgHeight = softBodyCM(innerstate.cloth->softBody.get()).z();
            //cout << "\t\taverage node height: " << avgHeight << '\n';
            float hullArea = projectedConvexHullArea(innerstate.cloth->softBody.get());
            //cout << "\t\tarea of convex hull of projected vertices: " << hullArea << '\n';

            float val = -hullArea;

            OptimalSpec &inneroptimal = threadoptimal[omp_get_thread_num()];
            if (val < inneroptimal.val) {
                inneroptimal.val = val;
                inneroptimal.actionspec = ac.spec;
                inneroptimal.state = innerstate;
            }
        } // end omp for
    } // end omp parallel

    // now find the best action from all the threads
    OptimalSpec optimal;
    optimal.val = numeric_limits<float>::infinity();
    for (int i = 0; i < numthreads; ++i)
        if (i == 0 || threadoptimal[i].val < optimal.val)
            optimal = threadoptimal[i];
    delete [] threadoptimal;

    cout << "optimal val: " << optimal.val << endl;
    boost::posix_time::ptime endTick(boost::posix_time::microsec_clock::local_time());
    std::cout << "\ttime: " << boost::posix_time::to_simple_string(endTick - begTick) << std::endl;

    newState = optimal.state;
    return optimal.actionspec;
}

/*NodeMoveAction::Spec flattenCloth_greedy_single(const StepState &initState, const vector<NodeMoveAction::Spec> &candspecs) {
    StepState state;
    return flattenCloth_greedy_single_internal(state, candspecs, state);
}*/

void flattenCloth_greedy(Scene &scene, BulletSoftObject::Ptr initCloth, NodeActionList &candidateActions, int steps, NodeActionList &out, bool actOnInputEnv) {
    // greedily find a list of actions that flattens the cloth
    vector<NodeMoveAction::Spec> candspecs;
    for (int i = 0; i < candidateActions.size(); ++i)
        candspecs.push_back(candidateActions[i].spec);

    StepState stepState;
    stepState.bullet.reset(new BulletInstance);
    stepState.bullet->setGravity(BulletConfig::gravity);
    stepState.osg.reset(new OSGInstance);
    stepState.fork.reset(new Fork(scene.env, stepState.bullet, stepState.osg));
    stepState.cloth = boost::static_pointer_cast<BulletSoftObject>(stepState.fork->forkOf(initCloth));
    BOOST_ASSERT(stepState.cloth);

    boost::posix_time::ptime begTick0(boost::posix_time::microsec_clock::local_time());
    for (int step = 0; step < steps; ++step) {
        cout << "step " << (step+1) << "/" << steps << '\n';
        scene.osg->root->removeChild(stepState.osg->root.get());

        if (actOnInputEnv) {
            StepState tmp;
            stepState.fork->env = scene.env;
            stepState.cloth = initCloth;
            out.add(flattenCloth_greedy_single_internal(stepState, candspecs, tmp, &scene,
                        1, 1, 1, 0.5));
        } else {
            out.add(flattenCloth_greedy_single_internal(stepState, candspecs, stepState, &scene,
                        1, 1, 1, 0.5));
        }

        scene.osg->root->addChild(stepState.osg->root.get());
        scene.draw();

        if (actOnInputEnv) {
            // replay?
            NodeMoveAction &a = out[step];
            a.setSoftBody(scene.env, initCloth->softBody.get());
            while (!a.done()) {
                a.step(BulletConfig::dt);
                scene.env->step(BulletConfig::dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
                scene.draw();
            }
        }
    }

    boost::posix_time::ptime endTick0(boost::posix_time::microsec_clock::local_time());
    cout << "time per step: " << boost::posix_time::to_simple_string((endTick0 - begTick0)/steps) << endl;
    cout << "total time: " << boost::posix_time::to_simple_string(endTick0 - begTick0) << endl;
}
