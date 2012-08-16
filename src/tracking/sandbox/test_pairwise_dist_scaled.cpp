#include <iostream>
#include <Eigen/Dense>
#include "utils/clock.h"
#include <cstdio>
#include <omp.h>
using namespace std;
using namespace Eigen;
MatrixXf pairwiseSquareDistScaled(const Eigen::MatrixXf& m_kf, const Eigen::MatrixXf& c_nf, const Eigen::MatrixXf sig_kf) {
	MatrixXf invsig2 = sig_kf.array().square().inverse();
	MatrixXf m2bysig2 = (m_kf.array().square() * invsig2.array()).rowwise().sum(); // k x 1
	MatrixXf c2bysig2 = invsig2 * c_nf.array().square().matrix().transpose(); // k x n
	MatrixXf mbysig2 = m_kf.array() * invsig2.array(); // k x f
	MatrixXf mcbysig2 = (mbysig2 * c_nf.transpose()); // k x n
	int N = c_nf.rows();
	return (-2) * mcbysig2 + m2bysig2.replicate(1,N) + c2bysig2;
}

MatrixXf pairwiseSquareDistScaled1(const Eigen::MatrixXf& m_kf, const Eigen::MatrixXf& c_nf, const Eigen::MatrixXf sig_kf) {
	MatrixXf out2(m_kf.rows(), c_nf.rows());
	ArrayXXf invsig2_kf = sig_kf.array().square().inverse();

	#pragma omp parallel for num_threads(4)
	for (int k = 0; k < m_kf.rows(); ++k) {
//		printf("We are %d\n", omp_get_num_threads());
		for (int n = 0; n < c_nf.rows(); ++n)
			out2(k,n) = ((m_kf.row(k) - c_nf.row(n)).array().square() * invsig2_kf.row(k)).sum();
	}
	return out2;
}

MatrixXf pairwiseSquareDistScaled2(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::MatrixXf sig_kf) {
	MatrixXf invVariances = sig_kf.array().square().inverse();

	int K = estPts.rows();
	int N = obsPts.rows();
	MatrixXf squaredDistsInvVariance(K,N);
#pragma omp parallel for num_threads(4)
	for (int k=0; k<K; k++) {
		squaredDistsInvVariance.row(k) = invVariances.row(k) * (obsPts.rowwise() - estPts.row(k)).transpose().array().square().matrix();
	}
	return squaredDistsInvVariance;

}



int main() {
	MatrixXf m_kf(3, 2);
	MatrixXf c_nf(4, 2);
	MatrixXf sig_kf(3, 2);
	m_kf << 1, 2, 3, 4, 5, 6;
	c_nf << 1, 2, 3, 4, 5, 6, 7, 8;
	sig_kf << 1, 2, 3, 4, 5, 6;

	MatrixXf out1 = pairwiseSquareDistScaled(m_kf, c_nf, sig_kf);
	MatrixXf out2 = pairwiseSquareDistScaled1(m_kf, c_nf, sig_kf);


	StartClock();
	double tStart;

	cout << out2 << endl;
	cout << out1 << endl;


	m_kf = MatrixXf::Random(500,8);
	c_nf = MatrixXf::Random(1000,8);
	sig_kf = MatrixXf::Random(500,8);

	MatrixXf q;

	int nTrials = 5;
	double tTotal0,tTotal1, tTotal2;

	StartClock();
	for (int trial=0; trial<nTrials; ++trial) {
		q = pairwiseSquareDistScaled(m_kf, c_nf, sig_kf);
	}
	tTotal0 = GetClock();

	StartClock();
	for (int trial=0; trial<nTrials; ++trial) {
		q = pairwiseSquareDistScaled1(m_kf, c_nf, sig_kf);
	}
	tTotal1 = GetClock();

	StartClock();
	for (int trial=0; trial<nTrials; ++trial) {
		q = pairwiseSquareDistScaled2(m_kf, c_nf, sig_kf);
	}
	tTotal2 = GetClock();


	printf("0,1,2: %.3f, %.3f, %.3f\n", tTotal0, tTotal1, tTotal2);


}
