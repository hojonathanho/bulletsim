#include "matching.h"
#include "my_matrix.h"
#include <gurobi_c++.h>
#include <my_assert.h>

using namespace Eigen;

GRBEnv env;


vector<int> invertIntFunc(const vector<int>& f, int maxVal) {
  vector<int> finv(maxVal, -1);
  for (int i=0; i < f.size(); i++) finv[f[i]] = i;
  return finv;
}

vector<int> matchHardMaximal(const MatrixXf& costs) {
  if (costs.rows() <= costs.cols()) return matchHardOneWay(costs);
  else return invertIntFunc(matchHardOneWay(costs.transpose()), costs.rows());
}

vector<int> matchHardOneWay(const MatrixXf& costs) {
  //  env.set(GRB_IntParam_OutputFlag, 0); 

  ENSURE(costs.rows() <= costs.cols());
  GRBModel model = GRBModel(env);

  // create matrix of variables
  MyMatrix<GRBVar> vars(costs.rows(),costs.cols());
  for (int iRow=0; iRow < costs.rows(); iRow++)
    for (int iCol=0; iCol < costs.cols(); iCol++)
      vars.at(iRow,iCol) = model.addVar(0,1,costs(iRow,iCol),GRB_BINARY);
  model.update();

  // row sum constraints
  vector<double> rowOfOnes(costs.cols(),1);
  for (int iRow=0; iRow<costs.rows(); iRow++) {
    const vector<GRBVar> rowOfVars = vars.getRow(iRow);
    GRBLinExpr expr;
    expr.addTerms(&rowOfOnes[0], &rowOfVars[0], rowOfOnes.size());
    model.addConstr(expr,GRB_EQUAL,1);
  }

  // column sum constraints
  vector<double> colOfOnes(costs.rows(),1);
  for (int iCol=0; iCol<costs.rows(); iCol++) {
    vector<GRBVar> colOfVars = vars.getCol(iCol);
    GRBLinExpr expr;
    expr.addTerms(&colOfOnes[0], &colOfVars[0], colOfOnes.size());
    model.addConstr(expr,GRB_LESS_EQUAL,1);
  }

  model.update();
  model.optimize();

  vector<int> matches(costs.rows());
  for (int iRow=0; iRow < costs.rows(); iRow++) {
    for (int iCol=0; iCol < costs.cols(); iCol++) {
      printf("(%i %i %f)\n",iRow, iCol, vars.at(iRow,iCol).get(GRB_DoubleAttr_X));
      if (vars.at(iRow,iCol).get(GRB_DoubleAttr_X) > 0) matches[iRow] = iCol;
    }
  }
  return matches;

}

SparseArray matchSoft(const MatrixXf& costs, float missingPenaltyA, float missingPenaltyB) {
  try {
    GRBModel model = GRBModel(env);

    int nA = costs.rows();
    int nB = costs.cols();
    double r = (double) nB / nA;

    MyMatrix<GRBVar> vars(nA,nB);
    for (int iA=0; iA<nA; iA++)
      for (int iB=0; iB<nB; iB++)
	vars.at(iA,iB) = model.addVar(0,GRB_INFINITY,costs(iA,iB),GRB_CONTINUOUS);


    vector<GRBVar> rowSums(nA);
    vector<GRBVar> colSums(nB);
    for (int iA=0; iA<nA; iA++) rowSums[iA]=model.addVar(-GRB_INFINITY, r, -missingPenaltyA, GRB_CONTINUOUS);
    for (int iB=0; iB<nB; iB++) colSums[iB]=model.addVar(-GRB_INFINITY, 1, -missingPenaltyB, GRB_CONTINUOUS);

    model.update();

    vector<double> colOnes(nA,1);
    vector<double> rowOnes(nB,1);
    for (int row=0; row<nA; row++) {
      vector<GRBVar> rowVars = vars.getRow(row);
      GRBLinExpr expr;
      expr.addTerms(&rowOnes[0], &rowVars[0], rowOnes.size());
      model.addConstr(expr, GRB_EQUAL, rowSums[row]);
    }
    for (int col=0; col<nB; col++) {
      vector<GRBVar> colVars = vars.getCol(col);
      GRBLinExpr expr;
      expr.addTerms(&colOnes[0], &colVars[0], colOnes.size());
      model.addConstr(expr, GRB_EQUAL, colSums[col]);
    }
    model.update();
    model.optimize();

    SparseArray out(nA);
    for (int row=0; row<nA; row++) {
      for (int col=0; col<nB; col++) {
	double p = vars.at(row,col).get(GRB_DoubleAttr_X);
	if (p>1e-4) out[row].push_back(IndVal(col,p));
      }
    }

    return out;
  }
  catch (GRBException e) {
    cout << "GRB error: " << e.getMessage() << endl;
    throw;
  }
}

