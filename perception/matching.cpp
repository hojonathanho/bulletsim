#include "matching.h"
#include "my_matrix.h"
#include <boost/foreach.hpp>
#include <my_assert.h>
using namespace std;
using namespace Eigen;


VectorXi invertIntFunc(const VectorXi& f, int maxVal) {
  VectorXi finv(maxVal, -1);
  for (int i=0; i < f.size(); i++) finv[f[i]] = i;
  return finv;
}

VectorXi matchHardMaximal(const MatrixXf& costs) {
  return matchHard(costs, 99999);
}

VectorXi matchHardOneWay(const MatrixXf& costs) {
    ENSURE(costs.rows() <= costs.cols());
    return matchHard(costs, 99999);
}

float matchingObjective(const VectorXi& f, const MatrixXf& D, float missingPenalty) {
    float cost = 0;
    for (int i=0; i < f.rows(); i++) {
        if (f[i] != -1) cost += D(i,f[i]);
        else cost += missingPenalty;
    }
    return cost;
}


vector<int> removeEl(const vector<int>& in, int x) {
    vector<int> out;
    BOOST_FOREACH(int el, in) if (el != x) out.push_back(el);
    return out;
}

vector< VectorXi > getIntFuncs(int m, const vector<int>& range) {
    vector<VectorXi> funcs;
    if (m==0) {
        VectorXi f(0,1);
        funcs.push_back(f);
    }
    else {
        BOOST_FOREACH(const VectorXi& f1, getIntFuncs(m-1,range)) {
            VectorXi f(m);
            f << -1, f1;        
            funcs.push_back(f);
        }
        BOOST_FOREACH(int x, range) {
            BOOST_FOREACH(const VectorXi& f1, getIntFuncs(m-1, removeEl(range, x))) {
                VectorXi f(m);
                f << x, f1;
                funcs.push_back(f);
            }
        }
    }
    return funcs;
}

vector< VectorXi > getMatchFuncs(int m, int n) {
    vector<int> range;
    for (int i=0; i<n; i++) range.push_back(i);
    return getIntFuncs(m, range);
}

VectorXi matchHard(const MatrixXf& dists, float missingPenalty) {    
    vector< VectorXi > fs = getMatchFuncs(dists.rows(), dists.cols());
    VectorXi scores(fs.size());
    for (int i=0; i < fs.size(); i++) {
        // cout << fs[i].transpose() << ": " << matchingObjective(fs[i], dists, missingPenalty) << endl;
        scores(i) = matchingObjective(fs[i], dists, missingPenalty);        
    }
    int iBest; scores.minCoeff(&iBest);
    return fs[iBest];    
}


// int main() {
//     vector<VectorXi> funcs = getMatchFuncs(3,2);
//     BOOST_FOREACH(VectorXi& v, funcs) cout << v.transpose() << endl;
//     MatrixXf D(3,2);
//     D << 10,20,  20,20,  20,10;
//     VectorXi M = matchHard(D, 15);
//     cout << "match" << endl;
//     cout << M.transpose() << endl;
//     
// }