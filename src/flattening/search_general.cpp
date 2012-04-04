#include "search_general.h"
//#include <omp.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <Wm5Core.h>
#include <Wm5Mathematics.h>
//#include <limits>
#include "graspingactions.h"
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

static float calcValue(Cloth::Ptr cloth) {
    return projectedConvexHullArea(cloth->psb()) + softBodyCM(cloth->psb()).z();
}

struct SearchContext {
    GraspingActionSpec spec; // the last action run
    GraspingActionContext ctx; // the context after the action was run
    float val; // value function output

    SearchContext() : spec(), ctx(), val(-1e99) { }
    SearchContext(const GraspingActionSpec &spec_, const GraspingActionContext &ctx_, float val_=-1e99) : spec(spec_), ctx(ctx_), val(val_) { }
};

static SearchContext tryAction(const GraspingActionContext &ctx, const GraspingActionSpec &spec) {
    GraspingActionContext forkCtx = ctx.fork();
    float value = -1e99;
    try {
        forkCtx.runAction(spec.createAction(forkCtx));
        value = calcValue(forkCtx.cloth);
    } catch (const GraspingActionFailed &) { }
    return SearchContext(spec, forkCtx, value);
}

static SearchContext flattenCloth_greedy_single_internal(const GraspingActionContext &initCtx, const GraspingActionSpec &prevAction) {
    SearchContext best;

    vector<GraspingActionSpec> succ = prevAction.genSuccessors(initCtx);
    for (int i = 0; i < succ.size(); ++i) {
        const GraspingActionSpec &spec = succ[i];
        cout << "TRYING ACTION (" << (1+i) << '/' << succ.size() << "): " << spec.specstr << endl;

        // if the action is a grab, then we have to go one level deeper
        // (a grab on its own is useless)
        // this won't be infinitely recursive since grabs' successors don't contain grabs
        SearchContext local;
/*        if (spec.type == GraspingActionSpec::GRAB) {
            SearchContext internalsc = tryAction(initCtx, spec);
            local = flattenCloth_greedy_single_internal(internalsc.ctx, internalsc.spec);
        } else {
            */
            local = tryAction(initCtx, spec);
       // }

        if (local.val > best.val)
            best = local;
    }

    return best;
}

GraspingActionSpec flattenCloth_greedy_single(const GraspingActionContext &initCtx, const GraspingActionSpec &prevAction) {
    return flattenCloth_greedy_single_internal(initCtx, prevAction).spec;
}

static SearchContext flattenCloth_single_internal(const SearchContext &init, int steps) {
    if (steps <= 0)
        return init;

    SearchContext best;
    best.val = init.val;

    vector<GraspingActionSpec> succ = init.spec.genSuccessors(init.ctx);
    for (int i = 0; i < succ.size(); ++i) {
        const GraspingActionSpec &spec = succ[i];
        cout << "TRYING ACTION (" << (1+i) << '/' << succ.size() << "): " << spec.specstr << endl;

        SearchContext local = tryAction(init.ctx, spec);
        SearchContext future = flattenCloth_single_internal(local, steps - 1);
        local.val = future.val;
        if (future.val > best.val)
            best = local;
    }

    return best;
}

GraspingActionSpec flattenCloth_single(const GraspingActionContext &initCtx, const GraspingActionSpec &prevAction, int steps) {
    return flattenCloth_single_internal(SearchContext(prevAction, initCtx), steps).spec;
}
