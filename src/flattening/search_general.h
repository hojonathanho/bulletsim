#ifndef __FL_SEARCH_GENERAL_H__
#define __FL_SEARCH_GENERAL_H__

#include "graspingactions.h"

GraspingActionSpec flattenCloth_greedy_single(
        const GraspingActionContext &initCtx,
        const GraspingActionSpec &prevAction=GraspingActionSpec());
//void flattenCloth_greedy(GraspingActionContext &ctx, int steps, vector<GraspingActionSpec> &out);

#endif // __FL_SEARCH_GENERAL_H__
