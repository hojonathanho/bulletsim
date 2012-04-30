#include <omp.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <Wm5Core.h>
#include <Wm5Mathematics.h>
//#include <limits>
#include "search_general.h"
#include "graspingactions.h"
using std::numeric_limits;

static const float MIN_VALUE = -1e99;

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

static bool clothInTableBounds(GraspingActionContext &ctx) {
    btSoftBody *psb = ctx.cloth->psb();
    for (int i = 0; i < psb->m_nodes.size(); ++i) {
        const btVector3 &p = psb->m_nodes[i].m_x;
        btVector3 q = ctx.table.trans.inverse() * p;
        if (abs(q.x()) > ctx.table.halfExtents.x() ||
            abs(q.y()) > ctx.table.halfExtents.y())
            return false;
    }
    return true;
}

static float calcValue(GraspingActionContext &ctx) {
    if (!clothInTableBounds(ctx))
        return MIN_VALUE;
    return projectedConvexHullArea(ctx.cloth->psb()) + softBodyCM(ctx.cloth->psb()).z();
}

struct SearchContext {
    GraspingActionSpec spec; // the last action run
    GraspingActionContext ctx; // the context after the action was run
    float val; // value function output

    SearchContext() : spec(), ctx(), val(MIN_VALUE) { }
    SearchContext(const GraspingActionSpec &spec_, const GraspingActionContext &ctx_, float val_=MIN_VALUE) : spec(spec_), ctx(ctx_), val(val_) { }
};

static SearchContext tryAction(const GraspingActionContext &ctx, const GraspingActionSpec &spec, bool debugDraw=false) {
    GraspingActionContext forkCtx = ctx.fork();
    float value = MIN_VALUE;
    try {
        forkCtx.runAction(spec.createAction(forkCtx), debugDraw);
        value = calcValue(forkCtx);
    } catch (const GraspingActionFailed &e) {
        cout << "WARNING: GraspingActionFailed: " << e.what() << endl;
        return SearchContext();
    }
    return SearchContext(spec, forkCtx, value);
}

static SearchContext flattenCloth_greedy_single_internal(const GraspingActionContext &initCtx, const GraspingActionSpec &prevAction) {
    vector<GraspingActionSpec> succ = prevAction.genSuccessors(initCtx);

    vector<SearchContext> threadbest;
    int numthreads;
    #pragma omp parallel shared(threadbest, numthreads)
    {
        #pragma omp single
        {
            numthreads = omp_get_num_threads();
            threadbest.resize(numthreads);
        }

        #pragma omp for
        for (int i = 0; i < succ.size(); ++i) {
            const GraspingActionSpec &spec = succ[i];
            cout << "TRYING ACTION (" << (1+i) << '/' << succ.size() << "): " << spec.specstr << endl;
            SearchContext local = tryAction(initCtx, spec, numthreads == 1);
            cout << "local val " << local.val << endl;
            SearchContext &best = threadbest[omp_get_thread_num()];
            if (local.val > best.val)
                best = local;
        } // end omp for
    } // end omp parallel

    int totalbest = 0;
    for (int i = 1; i < numthreads; ++i)
        if (threadbest[i].val > threadbest[totalbest].val)
            totalbest = i;
    return threadbest[totalbest];
}

GraspingActionSpec flattenCloth_greedy_single(const GraspingActionContext &initCtx, const GraspingActionSpec &prevAction) {
    return flattenCloth_greedy_single_internal(initCtx, prevAction).spec;
}

static SearchContext flattenCloth_single_internal(const SearchContext &init, int depth, int currdepth) {
    if (currdepth >= depth)
        return init;

    vector<SearchContext> threadbest;
    int numthreads;

    vector<GraspingActionSpec> succ = init.spec.genSuccessors(init.ctx);

    #pragma omp parallel if(currdepth == 0) shared(threadbest, numthreads)
    {
        #pragma omp single
        {
            numthreads = omp_get_num_threads();
            threadbest.resize(numthreads);
        }

        #pragma omp for
        for (int i = 0; i < succ.size(); ++i) {
            const GraspingActionSpec &spec = succ[i];
            cout << currdepth << '/' << depth << " >> TRYING ACTION (" << (1+i) << '/' << succ.size() << "): " << spec.specstr << endl;

            SearchContext local = tryAction(init.ctx, spec, false);
            SearchContext future = flattenCloth_single_internal(local, depth, currdepth + 1);
            local.val = future.val;
            SearchContext &best = threadbest[omp_get_thread_num()];
            if (future.val > best.val)
                best = local;
        }
    }

    int totalbest = 0;
    for (int i = 1; i < numthreads; ++i)
        if (threadbest[i].val > threadbest[totalbest].val)
            totalbest = i;
    return threadbest[totalbest];
}

GraspingActionSpec flattenCloth_single(const GraspingActionContext &initCtx, const GraspingActionSpec &prevAction, int depth) {
    return flattenCloth_single_internal(SearchContext(prevAction, initCtx), depth, 0).spec;
}
