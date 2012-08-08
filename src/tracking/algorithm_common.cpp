#include "algorithm_common.h"
#include "sparse_utils.h"
#include "utils_tracking.h"
#include "utils/conversions.h"
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "config_tracking.h"
#include "simulation/util.h"
#include "utils/testing.h"

using namespace Eigen;
using namespace std;

Eigen::MatrixXf calculateNodesNaive(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::MatrixXf& pZgivenC) {
	int K = estPts.rows(); //nodes
	int N = obsPts.rows(); //observations
	int F = estPts.cols(); //features

	assert(obsPts.cols() == F);
	assert(pZgivenC.rows() == K);
	assert(pZgivenC.cols() == N);

	MatrixXf nodes(K,F);
	for (int k=0; k<K; k++) {
		if (pZgivenC.row(k).sum() == 0.0) { //if visibility is zero, then the node position remains unchanged
			nodes.row(k) = estPts.row(k);
		} else {
			nodes.row(k) = MatrixXf::Zero(1,F);
			for (int n=0; n<N; n++)
				nodes.row(k) += pZgivenC(k,n)*obsPts.row(n);
			nodes.row(k) /= pZgivenC.row(k).sum();
		}
	}
	return nodes;
}

Eigen::MatrixXf calculateNodes(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::MatrixXf& pZgivenC) {
	int K = pZgivenC.rows(); //nodes
	int N = pZgivenC.cols(); //observations
	int F = obsPts.cols(); //features

	assert(obsPts.rows() == N);

	MatrixXf nodes(K,F);
	nodes = pZgivenC * obsPts;
	VectorXf invDenom = pZgivenC.rowwise().sum().array().inverse();
	for (int f=0; f<F; f++)
		nodes.col(f) = nodes.col(f).cwiseProduct(invDenom);
	return nodes;
}

Eigen::MatrixXf calculateStdevNaive(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::MatrixXf& pZgivenC, const Eigen::VectorXf& dPrior, const float& nuPrior) {
	int K = estPts.rows(); //nodes
	int N = obsPts.rows(); //observations
	int F = estPts.cols(); //features

	assert(obsPts.cols() == F);
	assert(pZgivenC.rows() == K);
	assert(pZgivenC.cols() == N);
	assert(dPrior.size() == F);

	MatrixXf variance(K,F);
	for (int k=0; k<K; k++) {
		for (int f=0; f<F; f++) {
			float nom = dPrior(f)*dPrior(f)*nuPrior;
			float denom = nuPrior;
			for (int n=0; n<N; n++) {
				nom += pZgivenC(k,n) * pow((obsPts(n,f) - estPts(k,f)),2);
				denom += pZgivenC(k,n);
			}
			variance(k,f) = nom/denom;
		}
	}
	return variance.array().sqrt();
}

Eigen::MatrixXf calculateStdev(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::MatrixXf& pZgivenC, const Eigen::VectorXf& dPrior, const float& nuPrior) {
	int K = estPts.rows(); //nodes
	int N = obsPts.rows(); //observations
	int F = estPts.cols(); //features

	assert(obsPts.cols() == F);
	assert(pZgivenC.rows() == K);
	assert(pZgivenC.cols() == N);
	assert(dPrior.size() == F);

	MatrixXf variance(K,F);
	for (int f=0; f<F; f++) {
		MatrixXf sqdists_f(K,N);// = pairwiseSquareDist(estPts.col(f), obsPts.col(f));
		for (int k=0; k<K; k++)
			sqdists_f.row(k) = (obsPts.col(f).array() - estPts(k,f)).square();
		variance.col(f) = pZgivenC.cwiseProduct(sqdists_f).rowwise().sum();
	}
	variance = variance.rowwise() + ((VectorXf) dPrior.array().square()).transpose()*nuPrior;
	VectorXf invDenom = (pZgivenC.rowwise().sum().array() + nuPrior).inverse();
	for (int f=0; f<F; f++)
		variance.col(f) = variance.col(f).cwiseProduct(invDenom);
	return variance.array().sqrt();
}

// gamma(z_nk) = p(z_k = 1 | c_n)
Eigen::MatrixXf calculateResponsibilitiesNaive(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::MatrixXf& stdev, const Eigen::VectorXf& pVis, const Eigen::VectorXf& outlierDist, const Eigen::VectorXf& outlierStdev) {
	int K = estPts.rows(); //nodes
	int N = obsPts.rows(); //observations
	int F = estPts.cols(); //features

	assert(obsPts.cols() == F);
	assert(stdev.rows() == K);
	assert(stdev.cols() == F);
	assert(pVis.size() == K);
	assert(outlierDist.size() <= F);

	MatrixXf invStdev = stdev.array().inverse();
	MatrixXf invVariances = invStdev.array().square();

	MatrixXf squaredDistsInvVariance(K+1,N);
	for (int k=0; k<K; k++) {
		for (int n=0; n<N; n++) {
			VectorXf diff = (obsPts.row(n) - estPts.row(k)).transpose();
			squaredDistsInvVariance(k,n) = diff.transpose() * invVariances.row(k).asDiagonal() * diff;
		}
	}
	VectorXf outlierInvStdev = outlierStdev.array().inverse();
	VectorXf outlierInvVariances = outlierInvStdev.array().square();
	for (int n=0; n<N; n++)
		squaredDistsInvVariance(K,n) = outlierDist.transpose() * outlierInvVariances.asDiagonal() * outlierDist;

	MatrixXf pZgivenC_exp_part = (-0.5*squaredDistsInvVariance).array().exp();
	VectorXf sqrtDetInvVariances = invStdev.rowwise().prod();
	MatrixXf pZgivenC(K+1,N);
	for (int k=0; k<K; k++)
		pZgivenC.row(k) = sqrtDetInvVariances(k) * pZgivenC_exp_part.row(k) * pVis(k);
	pZgivenC.row(K) = outlierInvStdev.prod() * pZgivenC_exp_part.row(K);

	//normalize cols
	for (int n=0; n<pZgivenC.cols(); n++)
		pZgivenC.col(n) /= pZgivenC.col(n).sum();

	//iteratively normalize rows and columns. columns are normalized to one and rows are normalized to the visibility term.
	for (int i=0; i<TrackingConfig::normalizeIter; i++) {
		//normalize rows
		//FIXME does the k+1 row need to be normalized?
		for (int k=0; k<K; k++) {
			if (pVis(k) == 0) {
				pZgivenC.row(k) = VectorXf::Zero(N);
			} else {
				pZgivenC.row(k) /= pZgivenC.row(k).sum();
			}
		}
		//pZgivenC.row(K) /= pZgivenC.row(K).sum();

		//normalize cols
		for (int n=0; n<pZgivenC.cols(); n++)
			pZgivenC.col(n) /= pZgivenC.col(n).sum();
	}

	infinityDebug(pZgivenC, "pZgivenC");
	assert(isFinite(pZgivenC));
	assert(pZgivenC.rows() == K+1);
	assert(pZgivenC.cols() == N);
	return pZgivenC.topRows(K);
}

// gamma(z_nk) = p(z_k = 1 | c_n)
Eigen::MatrixXf calculateResponsibilities(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::MatrixXf& stdev, const Eigen::VectorXf& pVis, const Eigen::VectorXf& outlierDist, const Eigen::VectorXf& outlierStdev) {
	int K = estPts.rows(); //nodes
	int N = obsPts.rows(); //observations
	int F = estPts.cols(); //features

	assert(obsPts.cols() == F);
	assert(stdev.rows() == K);
	assert(stdev.cols() == F);
	assert(pVis.size() == K);
	assert(outlierDist.size() <= F);

	MatrixXf invStdev = stdev.array().inverse();
	MatrixXf invVariances = invStdev.array().square();

	MatrixXf squaredDistsInvVariance(K+1,N);
	for (int k=0; k<K; k++) {
		squaredDistsInvVariance.row(k) = invVariances.row(k) * (obsPts.rowwise() - estPts.row(k)).transpose().array().square().matrix();
	}
	VectorXf outlierInvStdev = outlierStdev.array().inverse();
	VectorXf outlierInvVariances = outlierInvStdev.array().square();
	squaredDistsInvVariance.row(K) = outlierInvVariances.dot(outlierDist.array().square().matrix()) * VectorXf::Ones(N).transpose();

	MatrixXf pZgivenC_exp_part = (-0.5*squaredDistsInvVariance).array().exp();
	VectorXf sqrtDetInvVariances = invStdev.rowwise().prod();
	MatrixXf pZgivenC(K+1,N);
	for (int k=0; k<K; k++)
		pZgivenC.row(k) = sqrtDetInvVariances(k) * pZgivenC_exp_part.row(k) * pVis(k);
	pZgivenC.row(K) = outlierInvStdev.prod() * pZgivenC_exp_part.row(K);

	//normalize cols
	pZgivenC = pZgivenC * ((VectorXf) pZgivenC.colwise().sum().array().inverse()).asDiagonal();

	//iteratively normalize rows and columns. columns are normalized to one and rows are normalized to the visibility term.
	for (int i=0; i<TrackingConfig::normalizeIter; i++) {
		//normalize rows
		//FIXME does the k+1 row need to be normalized?
		for (int k=0; k<K; k++) {
			if (pVis(k) == 0) {
				pZgivenC.row(k) = VectorXf::Zero(N);
			} else {
				pZgivenC.row(k) /= pZgivenC.row(k).sum();
			}
		}
		//pZgivenC.row(K) /= pZgivenC.row(K).sum();

		//normalize cols
		pZgivenC = pZgivenC * ((VectorXf) pZgivenC.colwise().sum().array().inverse()).asDiagonal();
	}

	assert(isFinite(pZgivenC));
	assert(pZgivenC.rows() == K+1);
	assert(pZgivenC.cols() == N);
	return pZgivenC.topRows(K);
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////// DEPRECATED //////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

void estimateCorrespondence(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& stdev, const Eigen::VectorXf& pVis,
  const Eigen::MatrixXf& obsPts, const Eigen::VectorXf& outlierDist, Eigen::MatrixXf& pZgivenB, SparseMatrixf& corr) {
	assert((estPts.rows()+1) == stdev.rows());
	assert(estPts.cols() == stdev.cols());
	assert(estPts.cols() == obsPts.cols());
	assert(pVis.size() == estPts.rows());

	//cout << "estPts" << endl;
	//infinityDebug(estPts);
	//cout << "obsPts" << endl;
	//infinityDebug(obsPts);

	MatrixXf invStdev = stdev.array().inverse();
	MatrixXf invVariances = invStdev.array().square();
	MatrixXf squaredDistsInvVariance(estPts.rows()+1, obsPts.rows());

//	//The loop below is equivalent to this loops
//	for (int i=0; i<estPts.rows(); i++) {
//		for (int j=0; j<obsPts.rows(); j++) {
//			VectorXf diff = (estPts.row(i) - obsPts.row(j)).transpose();
//			sqdistsSigma(i,j) = diff.transpose() * invVariances.row(i).asDiagonal() * diff;
//		}
//	}
	for (int i=0; i<estPts.rows(); i++) {
		squaredDistsInvVariance.row(i) = invVariances.row(i) * (obsPts.rowwise() - estPts.row(i)).transpose().array().square().matrix();
	}
	//squaredDistsInvVariance.row(estPts.rows()) = invVariances.row(estPts.rows()).dot(outlierDist.array().square().matrix()) * VectorXf::Ones(squaredDistsInvVariance.cols()).transpose();
	//intuition: when an observed point is farther than space_distance = outlierDist.topRows(3).norm() from an estimated point (node), then that observed point is considered an outlier
	//special case: when pointOutlierDist = outlierDist(0) = outlierDist(1) = outlierDist(2), space_dist = sqrt(3) * pointOutlierDist
	squaredDistsInvVariance.row(estPts.rows()) = invVariances.leftCols(3).row(estPts.rows()).dot(outlierDist.topRows(3).array().square().matrix()) * VectorXf::Ones(squaredDistsInvVariance.cols()).transpose();

	MatrixXf pBgivenZ_unscaled = (-squaredDistsInvVariance).array().exp();
	VectorXf negSqrtDetVariances = invStdev.rowwise().prod();

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
				//cout << pZgivenB.rowwise().sum() << endl;
				pZgivenB.row(i) = VectorXf::Zero(pZgivenB.cols());
			}
		}

		//normalize cols
		pZgivenB = pZgivenB * ((VectorXf) pZgivenB.colwise().sum().array().inverse()).asDiagonal();
	}

	//corr = toSparseMatrix(pZgivenB.topRows(estPts.rows()), .1);

//	int r=0;
//	for (int row=0; row < pZgivenB.rows(); row++) {
//		int c=0;
//		int first_col=-1;
//		for (int col=0; col<pZgivenB.cols(); col++) {
//			if (!isfinite(pZgivenB(row,col))) {
//				if (first_col<0) first_col = col;
//				c++;
//			}
//		}
//		if (c>0) {
//			printf("%d/%d infinite cols \tpZgivenB(%d,%d)=%f \tpVis_kp1(%d)=%f\n", c, pZgivenB.cols(), row, first_col, pZgivenB(row,first_col), row, pVis_kp1(row));
//			r++;
//		}
//	}
//	if (r>0) printf("%d/%d infinite rows\n", r, pZgivenB.rows());
	assert(isFinite(pZgivenB));
	assert(pZgivenB.rows() == (estPts.rows())+1);
	assert(pZgivenB.cols() == obsPts.rows());
	pZgivenB = pZgivenB.topRows(estPts.rows());
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
