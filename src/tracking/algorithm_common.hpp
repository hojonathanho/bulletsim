//#include "sparse_utils.h"
//#include "utils_tracking.h"
//#include "utils/conversions.h"
#include <fstream>
#include "config_tracking.h"
//#include "simulation/util.h"
#include "utils/testing.h"

using namespace Eigen;
using namespace std;

//// gamma(z_nk) = p(z_k = 1 | c_n)
//Eigen::MatrixXf calculateResponsibilitiesNaive(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::MatrixXf& stdev, const Eigen::VectorXf& pVis, const Eigen::VectorXf& outlierDist, const Eigen::VectorXf& outlierStdev) {
//	int K = estPts.rows(); //nodes
//	int N = obsPts.rows(); //observations
//	int F = estPts.cols(); //features
//
//	assert(obsPts.cols() == F);
//	assert(stdev.rows() == K);
//	assert(stdev.cols() == F);
//	assert(pVis.size() == K);
//	assert(outlierDist.size() <= F);
//
//	MatrixXf invStdev = stdev.array().inverse();
//	MatrixXf invVariances = invStdev.array().square();
//
//	MatrixXf squaredDistsInvVariance(K+1,N);
//	for (int k=0; k<K; k++) {
//		for (int n=0; n<N; n++) {
//			VectorXf diff = (obsPts.row(n) - estPts.row(k)).transpose();
//			squaredDistsInvVariance(k,n) = diff.transpose() * invVariances.row(k).asDiagonal() * diff;
//		}
//	}
//	VectorXf outlierInvStdev = outlierStdev.array().inverse();
//	VectorXf outlierInvVariances = outlierInvStdev.array().square();
//	for (int n=0; n<N; n++)
//		squaredDistsInvVariance(K,n) = outlierDist.transpose() * outlierInvVariances.asDiagonal() * outlierDist;
//
//	MatrixXf pZgivenC_exp_part = (-0.5*squaredDistsInvVariance).array().exp();
//	VectorXf sqrtDetInvVariances = invStdev.rowwise().prod();
//	MatrixXf pZgivenC(K+1,N);
//	for (int k=0; k<K; k++)
//		pZgivenC.row(k) = sqrtDetInvVariances(k) * pZgivenC_exp_part.row(k) * pVis(k);
//	pZgivenC.row(K) = outlierInvStdev.prod() * pZgivenC_exp_part.row(K);
//
//	//normalize cols
//	for (int n=0; n<pZgivenC.cols(); n++)
//		pZgivenC.col(n) /= pZgivenC.col(n).sum();
//
//	//iteratively normalize rows and columns. columns are normalized to one and rows are normalized to the visibility term.
//	for (int i=0; i<TrackingConfig::normalizeIter; i++) {
//		//normalize rows
//		//FIXME does the k+1 row need to be normalized?
//		for (int k=0; k<K; k++) {
//			if (pVis(k) == 0) {
//				pZgivenC.row(k) = VectorXf::Zero(N);
//			} else {
//				pZgivenC.row(k) /= pZgivenC.row(k).sum();
//			}
//		}
//		//pZgivenC.row(K) /= pZgivenC.row(K).sum();
//
//		//normalize cols
//		for (int n=0; n<pZgivenC.cols(); n++)
//			pZgivenC.col(n) /= pZgivenC.col(n).sum();
//	}
//
//	//infinityDebug(pZgivenC, "pZgivenC");
//	assert(isFinite(pZgivenC));
//	assert(pZgivenC.rows() == K+1);
//	assert(pZgivenC.cols() == N);
//	return pZgivenC.topRows(K);
//}

// gamma(z_nk) = p(z_k = 1 | c_n)

template <typename T>
Eigen::Matrix<T, Dynamic, Dynamic> calculateResponsibilitiesT(
		const Eigen::Matrix< T , Dynamic , Dynamic >& estPts,
		const Eigen::Matrix< T , Dynamic , Dynamic >& obsPts,
		const Eigen::Matrix< T , Dynamic , Dynamic >& stdev,
		const Eigen::Matrix< T , Dynamic , 1 >& pVis,
		const Eigen::Matrix< T , Dynamic , 1 >& outlierDist,
		const Eigen::Matrix< T , Dynamic , 1 >& outlierStdev )
{
	typedef typename Eigen::Matrix< T , Dynamic , 1 > VectorXt;
	typedef typename Eigen::Matrix< T , Dynamic , Dynamic > MatrixXt;

	int K = estPts.rows(); //nodes
	int N = obsPts.rows(); //observations
	int F = estPts.cols(); //features

	assert(obsPts.cols() == F);
	assert(stdev.rows() == K);
	assert(stdev.cols() == F);
	assert(pVis.size() == K);
	assert(outlierDist.size() <= F);

	MatrixXt invStdev = stdev.array().inverse();
	MatrixXt invVariances = invStdev.array().square();

	MatrixXt squaredDistsInvVariance(K+1,N);
	for (int k=0; k<K; k++) {
		squaredDistsInvVariance.row(k) = invVariances.row(k) * (obsPts.rowwise() - estPts.row(k)).transpose().array().square().matrix();
	}
	VectorXt outlierInvStdev = outlierStdev.array().inverse();
	VectorXt outlierInvVariances = outlierInvStdev.array().square();
	squaredDistsInvVariance.row(K) = outlierInvVariances.dot(outlierDist.array().square().matrix()) * VectorXt::Ones(N).transpose();

	MatrixXt pZgivenC_exp_part = (-0.5*squaredDistsInvVariance).array().exp();
	VectorXt sqrtDetInvVariances = invStdev.rowwise().prod();

	MatrixXt pZgivenC(K+1,N);
	for (int k=0; k<K; k++)
		pZgivenC.row(k) = sqrtDetInvVariances(k) * pZgivenC_exp_part.row(k) * pVis(k);
	pZgivenC.row(K) = outlierInvStdev.prod() * pZgivenC_exp_part.row(K);

	//normalize cols
	pZgivenC = pZgivenC * ((VectorXt) pZgivenC.colwise().sum().array().inverse()).asDiagonal();

	//iteratively normalize rows and columns. columns are normalized to one and rows are normalized to the visibility term.
	for (int i=0; i<TrackingConfig::normalizeIter; i++) {
		//normalize rows
		//FIXME does the k+1 row need to be normalized?
		for (int k=0; k<K; k++) {
			if (pVis(k) == 0) {
				pZgivenC.row(k) = VectorXt::Zero(N);
			} else {
				pZgivenC.row(k) /= pZgivenC.row(k).sum();
			}
		}
		//pZgivenC.row(K) /= pZgivenC.row(K).sum();

		//normalize cols
		pZgivenC = pZgivenC * ((VectorXt) pZgivenC.colwise().sum().array().inverse()).asDiagonal();
	}

	//assert(isFinite(pZgivenC));
	assert(pZgivenC.rows() == K+1);
	assert(pZgivenC.cols() == N);
	return pZgivenC.topRows(K);
}
