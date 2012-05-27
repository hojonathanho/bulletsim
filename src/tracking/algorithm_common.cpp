#include "algorithm_common.h"
#include "sparse_utils.h"
#include "utils_tracking.h"
using namespace Eigen;
void estimateCorrespondence(const Eigen::MatrixXf& estPts, const Eigen::VectorXf& variances, const Eigen::VectorXf& pVis, 
  const Eigen::MatrixXf& obsPts, float pBandOutlier, SparseMatrixf& corr) {

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
    assert(isFinite(pZgivenB));
    corr = toSparseMatrix(pZgivenB, .1);
}
