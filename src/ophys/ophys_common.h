#ifndef __OPTPHYSICS_COMMON_H__
#define __OPTPHYSICS_COMMON_H__

#include <vector>
#include <boost/multi_array.hpp>
#include <gurobi_c++.h>

namespace ophys {

typedef std::vector<GRBVar *> VarPVec;
typedef std::vector<GRBVar> VarVec;
typedef std::vector<GRBTempConstr> ConstrVec;
typedef boost::multi_array<GRBVar, 2> VarMatrix;

inline GRBQuadExpr square(const GRBLinExpr &e) { return e*e; }

template<typename T>
inline T square(const T& e) { return e*e; }

} // namespace ophys

#endif // __OPTPHYSICS_COMMON_H__
