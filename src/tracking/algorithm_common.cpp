#include "algorithm_common.h"
#include "sparse_utils.h"
#include "utils_tracking.h"

#include <fstream>

using namespace Eigen;
using namespace std;


void estimateCorrespondence(const Eigen::MatrixXf& estPts, const Eigen::VectorXf& variances, const Eigen::VectorXf& pVis, 
  const Eigen::MatrixXf& obsPts, float pBandOutlier, SparseMatrixf& corr) {


//	ofstream out("/tmp/junk.txt");
//	out << "estPts" << endl << estPts << endl;
//	out << "obsPts" << endl << obsPts << endl;
//	out << "variances" << endl << variances << endl;
//	out << "pBAnd" << endl << pBandOutlier << endl;

    VectorXf invVariances = variances.array().inverse();
    MatrixXf sqdists = pairwiseSquareDist(estPts, obsPts);
    MatrixXf tmp1 = ((-invVariances).asDiagonal() * sqdists).array().exp();
    VectorXf tmp2 = invVariances.array().pow(1.5);
    MatrixXf pBgivenZ_unnormed = tmp2.asDiagonal() * tmp1;
    MatrixXf pBandZ_unnormed = pVis.asDiagonal()*pBgivenZ_unnormed;
    VectorXf pB_unnormed = pBandZ_unnormed.colwise().sum();
    VectorXf pBorOutlier_unnormed = (pB_unnormed.array() + pBandOutlier);
//    float loglik = pBorOutlier_unnormed.sum();
    MatrixXf pZgivenB = pBandZ_unnormed * pBorOutlier_unnormed.asDiagonal().inverse();
    corr = toSparseMatrix(pZgivenB, .1);

#define DBG_SAVE_VAR(varname)\
	do {\
	  ofstream outfile("/tmp/"#varname".txt");\
	  outfile << varname << endl;\
	} while(0)
//
//    DBG_SAVE_VAR(obsPts);
//    DBG_SAVE_VAR(estPts);
//    DBG_SAVE_VAR(pVis);
//    DBG_SAVE_VAR(pZgivenB);
//    DBG_SAVE_VAR(variances);
//    DBG_SAVE_VAR(corr);



    assert(isFinite(pZgivenB));

}
