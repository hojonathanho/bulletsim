#ifndef __OPTPHYSICS_COMMON_H__
#define __OPTPHYSICS_COMMON_H__

#include <gurobi_c++.h>

namespace ophys {

typedef vector<GRBVar *> VarPVec;
typedef vector<GRBVar> VarVec;
typedef vector<GRBTempConstr> ConstrVec;

inline GRBQuadExpr square(const GRBLinExpr &e) { return e*e; }

} // namespace ophys

#endif // __OPTPHYSICS_COMMON_H__
