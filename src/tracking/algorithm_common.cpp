#include "algorithm_common.h"
#include "sparse_utils.h"
#include "utils_tracking.h"
#include <fstream>

using namespace Eigen;
using namespace std;

Eigen::MatrixXf calcSigs(const SparseMatrixf& corr, const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, Eigen::VectorXf priorDist, float priorCount) {
	assert(corr.rows() == estPts.rows());
	assert(corr.cols() == obsPts.rows());
	assert(estPts.cols() == obsPts.cols());
	assert(estPts.cols() == priorDist.size());

	MatrixXf Z = corr;
	MatrixXf sigs(estPts.rows(), estPts.cols());
	for (int k=0; k<estPts.rows(); k++) {
		for (int f=0; f<estPts.cols(); f++) {
			float nom = priorDist(f)*priorDist(f)*priorCount;
			//float nom = 3*priorDist*priorDist*priorCount;
			float denom = priorCount;
			for (int n=0; n<corr.cols(); n++) {
				nom += pow((estPts(k,f) - obsPts(n,f)),2)*Z(k,n);
				denom += Z(k,n);
			}
			//denom *= 3;
			sigs(k,f) = nom/denom;
		}
	}
	sigs = sigs.array().sqrt();
	return sigs;
}

//Same as calSigs. This implementation uses Eigen but it's slower when used for the rope.
Eigen::MatrixXf calcSigsEigen(const SparseMatrixf& corr, const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, Eigen::VectorXf priorDist, float priorCount) {
	MatrixXf Z = corr;
  MatrixXf sigs(estPts.rows(), estPts.cols());
	for (int f=0; f<sigs.cols(); f++) {
		MatrixXf sqdists_f = pairwiseSquareDist(estPts.col(f), obsPts.col(f));
		sigs.col(f) = Z.cwiseProduct(sqdists_f).rowwise().sum();
	}
	sigs = sigs.rowwise() + ((VectorXf) priorDist.array().square()).transpose()*priorCount;
	VectorXf denom = Z.rowwise().sum().array() + priorCount;
	sigs = sigs.array() / denom.replicate(1, sigs.cols()).array();
	sigs = sigs.array().sqrt();
	return sigs;
}

void estimateCorrespondence(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& sigma, const Eigen::VectorXf& pVis,
  const Eigen::MatrixXf& obsPts, float pBandOutlier, SparseMatrixf& corr) {
	assert(estPts.rows() == sigma.rows());
	assert(estPts.cols() == sigma.cols());
	assert(estPts.cols() == obsPts.cols());
	assert(pVis.size() == estPts.rows());

	MatrixXf invSigma = sigma.array().inverse();

	MatrixXf invVariances = invSigma.array().square();
	MatrixXf sqdistsSigma(estPts.rows(), obsPts.rows());
	for (int i=0; i<estPts.rows(); i++) {
		for (int j=0; j<obsPts.rows(); j++) {
			VectorXf diff = (estPts.row(i) - obsPts.row(j)).transpose();
			sqdistsSigma(i,j) = diff.transpose() * invVariances.row(i).asDiagonal() * diff;
		}
	}

	MatrixXf tmp1 = (-sqdistsSigma).array().exp();
	VectorXf tmp2 = invSigma.rowwise().prod();
	MatrixXf pBgivenZ_unnormed = tmp2.asDiagonal() * tmp1;
	MatrixXf pBandZ_unnormed = pVis.asDiagonal()*pBgivenZ_unnormed;
	VectorXf pB_unnormed = pBandZ_unnormed.colwise().sum();
	VectorXf pBorOutlier_unnormed = (pB_unnormed.array() + pBandOutlier);
//    float loglik = pBorOutlier_unnormed.sum();
	MatrixXf pZgivenB = pBandZ_unnormed * pBorOutlier_unnormed.asDiagonal().inverse();
	corr = toSparseMatrix(pZgivenB, .1);

//	MAT_DIMS(estPts);
//	MAT_DIMS(obsPts);
//	MAT_DIMS(pZgivenB);
//	MAT_DIMS(corr);

	assert(isFinite(pZgivenB));
	assert(corr.rows() == estPts.rows());
	assert(corr.cols() == obsPts.rows());
}

// Same as estimateCorrespondence IF all the rows of sigma are the same.
void estimateCorrespondenceSame(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& sigma, const Eigen::VectorXf& pVis,
  const Eigen::MatrixXf& obsPts, float pBandOutlier, SparseMatrixf& corr) {

		MatrixXf invSigma = sigma.array().inverse();

    // (x-u).transpose() * Sigma.inverse() * (x-u)
    // equivalent to
    // MatrixXf invVariances = invSigma.array().square();
    // MatrixXf sqdistsSigma(estPts.size(), estPts.size());
		// for (int i=0; i<estPts.size(); i++) {
		// 	 for (int j=0; j<estPts.size(); j++) {
		//		 VectorXf diff = (estPts.row(i) - obsPts.row(j));
		//		 sqdistsSigma(i,j) = diff.transpose() * invVariances.row(i).asDiagonal() * diff;
		//	 }
		// }
		// assumes that all the rows of sigma are the same. doesn't assume anything about elements in the rows.
    MatrixXf sqdistsSigma = pairwiseSquareDist(estPts*invSigma.row(0).asDiagonal(), obsPts*invSigma.row(0).asDiagonal());

    MatrixXf tmp1 = (-sqdistsSigma).array().exp();
    VectorXf tmp2 = invSigma.rowwise().prod();
    MatrixXf pBgivenZ_unnormed = tmp2.asDiagonal() * tmp1;
    MatrixXf pBandZ_unnormed = pVis.asDiagonal()*pBgivenZ_unnormed;
    VectorXf pB_unnormed = pBandZ_unnormed.colwise().sum();
    VectorXf pBorOutlier_unnormed = (pB_unnormed.array() + pBandOutlier);
//    float loglik = pBorOutlier_unnormed.sum();

    #define MAT_DIMS(mat)\
    	cout << #mat << " " << mat.rows() << " " << mat.cols() << endl;
        MAT_DIMS(estPts);
        MAT_DIMS(obsPts);
        //MAT_DIMS(pZgivenB);
        //MAT_DIMS(corr);

    MatrixXf pZgivenB = pBandZ_unnormed * pBorOutlier_unnormed.asDiagonal().inverse();
    corr = toSparseMatrix(pZgivenB, .1);



    assert(isFinite(pZgivenB));
}

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
