#include "algorithm_common.h"
#include "sparse_utils.h"
#include "utils_tracking.h"
#include "utils/conversions.h"
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "config_tracking.h"

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
			float denom = priorCount;
			for (int n=0; n<corr.cols(); n++) {
				nom += pow((estPts(k,f) - obsPts(n,f)),2)*Z(k,n);
				denom += Z(k,n);
			}
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
  const Eigen::MatrixXf& obsPts, const Eigen::VectorXf& outlierDist, Eigen::MatrixXf& pZgivenB, SparseMatrixf& corr) {
	assert((estPts.rows()+1) == sigma.rows());
	assert(estPts.cols() == sigma.cols());
	assert(estPts.cols() == obsPts.cols());
	assert(pVis.size() == estPts.rows());

	MatrixXf invSigma = sigma.array().inverse();
	MatrixXf invVariances = invSigma.array().square();
	MatrixXf sqDistsInvSigma(estPts.rows()+1, obsPts.rows());

//	//The loop below is equivalent to this loops
//	for (int i=0; i<estPts.rows(); i++) {
//		for (int j=0; j<obsPts.rows(); j++) {
//			VectorXf diff = (estPts.row(i) - obsPts.row(j)).transpose();
//			sqdistsSigma(i,j) = diff.transpose() * invVariances.row(i).asDiagonal() * diff;
//		}
//	}
	for (int i=0; i<estPts.rows(); i++) {
		sqDistsInvSigma.row(i) = invVariances.row(i) * (obsPts.rowwise() - estPts.row(i)).transpose().array().square().matrix();
	}
	//sqDistsInvSigma.row(estPts.rows()) = invVariances.row(estPts.rows()).dot(outlierDist.array().square().matrix()) * VectorXf::Ones(sqDistsInvSigma.cols()).transpose();
	//intuition: when an observed point is farther than space_distance = outlierDist.topRows(3).norm() from an estimated point (node), then that observed point is considered an outlier
	//special case: when pointOutlierDist = outlierDist(0) = outlierDist(1) = outlierDist(2), space_dist = sqrt(3) * pointOutlierDist
	sqDistsInvSigma.row(estPts.rows()) = invVariances.leftCols(3).row(estPts.rows()).dot(outlierDist.topRows(3).array().square().matrix()) * VectorXf::Ones(sqDistsInvSigma.cols()).transpose();

	MatrixXf pBgivenZ_unscaled = (-sqDistsInvSigma).array().exp();
	VectorXf negSqrtDetVariances = invSigma.rowwise().prod();

	VectorXf pVis_kp1(pVis.size()+1);
	pVis_kp1.topRows(pVis.size()) = pVis;
	pVis_kp1(pVis.size()) = 1.0;

	MatrixXf pBandZ_unnormed = pVis_kp1.cwiseProduct(negSqrtDetVariances).asDiagonal()*pBgivenZ_unscaled;
	VectorXf pB_unnormed = pBandZ_unnormed.colwise().sum();
	//normalize cols
	pZgivenB = pBandZ_unnormed * ((VectorXf) pB_unnormed.array().inverse()).asDiagonal();

	//iteratively normalize rows and columns. columns are normalized to one and rows are normalized to the visibility term.
	for (int i=0; i<TrackingConfig::normalizeIter; i++) {
		//normalize rows
		pZgivenB = ((VectorXf) (pVis_kp1.array() * (pZgivenB.rowwise().sum().array()+TrackingConfig::epsilon).inverse())).asDiagonal() * pZgivenB;

		for (int i=0; i<pZgivenB.rows()-1; i++) {
			if (pVis_kp1(i) == 0) {
				pZgivenB.row(i) *= 0.0;
			}
		}

		//normalize cols
		pZgivenB = pZgivenB * ((VectorXf) pZgivenB.colwise().sum().array().inverse()).asDiagonal();
	}

	corr = toSparseMatrix(pZgivenB.topRows(estPts.rows()), .1);

	assert(isFinite(pZgivenB));
	for (int row=0; row < pZgivenB.rows(); row++)
		for (int col=0; col<pZgivenB.cols(); col++)
			if (!isfinite(pZgivenB(row,col))) {
				cout << "infinite" << endl;
				pZgivenB(row,col) = 0;
			}
	assert(pZgivenB.rows() == (estPts.rows())+1);
	assert(pZgivenB.cols() == obsPts.rows());
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////// The functions below are experimental only as they apply for special cases //////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

//Approximation of estimateCorrespondence using a kdtree. The correspondence of a estimated point is estimated by looking at the k nearest neighbors observation points.
void estimateCorrespondenceCloud (ColorCloudPtr cloud, const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& sigma, const Eigen::VectorXf& pVis,
	  const Eigen::MatrixXf& obsPts, float pBandOutlier, SparseMatrixf& corr, int K) {
	MatrixXf invSigma = sigma.array().inverse();
	MatrixXf invVariances = invSigma.array().square();
	MatrixXf pBgivenZ_unscaled = MatrixXf::Zero(estPts.rows(), obsPts.rows());

	pcl::KdTreeFLANN<ColorPoint> kdtree;
	kdtree.setInputCloud(cloud);
	ColorPoint searchPoint;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	for (int i=0; i<estPts.rows(); i++) {
		searchPoint = toColorPoint(estPts.block(i,0,1,3).transpose());
		pointIdxNKNSearch = std::vector<int> (K);
		pointNKNSquaredDistance = std::vector<float> (K);
		if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )	{
			for (size_t j = 0; j < pointIdxNKNSearch.size (); j++) {
				pBgivenZ_unscaled(i,j) = exp(-invVariances.row(i).dot((estPts.row(i) - obsPts.row(j)).array().square().matrix()));
			}
		}
	}

	VectorXf negSqrtDetVariances = invSigma.rowwise().prod();
	MatrixXf pBandZ_unnormed = pVis.cwiseProduct(negSqrtDetVariances).asDiagonal()*pBgivenZ_unscaled;

	VectorXf pB_unnormed = pBandZ_unnormed.colwise().sum();
	VectorXf pBorOutlier_unnormed = (pB_unnormed.array() + pBandOutlier);
//    float loglik = pBorOutlier_unnormed.sum();
	MatrixXf pZgivenB = pBandZ_unnormed * ((VectorXf) pBorOutlier_unnormed.array().inverse()).asDiagonal();
	corr = toSparseMatrix(pZgivenB, .1);
}

//Assumes that the variances of each feature of a particular node are the same.
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
