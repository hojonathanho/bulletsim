#ifndef _eigen_multivariate_normal_h
#define _eigen_multivariate_normal_h

#include <Eigen/Dense>

#include <math.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

/**
 *     We find the eigen-decomposition of the covariance matrix.
 *         We create a vector of normal samples scaled by the eigenvalues.
 *             We rotate the vector by the eigenvectors.
 *                 We add the mean.
 *                 */
using namespace Eigen; 


template<typename _Scalar>
class EigenMultivariateNormal
{
  boost::mt19937 rng;    // The uniform pseudo-random algorithm
  boost::normal_distribution<_Scalar> norm;  // The gaussian combinator
  boost::variate_generator<boost::mt19937&,boost::normal_distribution<_Scalar> >
    randN; // The 0-mean unit-variance normal generator

  MatrixXd rot;
  VectorXd scl;
  VectorXd mean;
  int _size; 

  public:
  EigenMultivariateNormal(const VectorXd &meanVec,
      const MatrixXd& covarMat)
    : randN(rng,norm)
  {
    randN.engine().seed(static_cast<unsigned int>(std::time(0)));

    assert(covarMat.rows() == covarMat.cols());
    assert(meanVec.rows() == covarMat.cols()); 
    _size = meanVec.rows(); 
    setCovar(covarMat);
    setMean(meanVec);
  }

  void setCovar(const MatrixXd& covarMat)
  {
    Eigen::SelfAdjointEigenSolver<MatrixXd>
      eigenSolver(covarMat);
    rot = eigenSolver.eigenvectors();
    scl = eigenSolver.eigenvalues();
    for (int ii=0;ii<_size;++ii) {
      scl(ii) = sqrt(scl(ii));
    }
  }

  void setMean(const MatrixXd& meanVec)
  {
    mean = meanVec;
  }

  VectorXd nextSample()
  {
    VectorXd sampleVec(_size);
    for (int ii=0;ii<_size;++ii) {
      sampleVec(ii) = randN()*scl(ii);
    }
    sampleVec = rot*sampleVec + mean;
    return sampleVec;
  }

  void generateSamples(MatrixXd& samples, int NS) { 
    samples = MatrixXd(_size, NS);
    for (int i = 0; i < NS; i++) samples.col(i) = nextSample();

  }

};

#endif
