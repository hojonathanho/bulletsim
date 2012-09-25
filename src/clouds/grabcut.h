/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "gcgraph.hpp"

/*
This is implementation of image segmentation algorithm GrabCut described in
"GrabCut — Interactive Foreground Extraction using Iterated Graph Cuts".
Carsten Rother, Vladimir Kolmogorov, Andrew Blake.
 */

/*
 GMM - Gaussian Mixture Model
*/
class GMM
{
public:
    static const int componentsCount = 5;

    GMM( cv::Mat& _model );
    double operator()( const cv::Vec3d color ) const;
    double operator()( int ci, const cv::Vec3d color ) const;
    int whichComponent( const cv::Vec3d color ) const;

    void initLearning();
    void addSample( int ci, const cv::Vec3d color );
    void endLearning();

private:
    void calcInverseCovAndDeterm( int ci );
    cv::Mat model;
    double* coefs;
    double* mean;
    double* cov;

    double inverseCovs[componentsCount][3][3];
    double covDeterms[componentsCount];

    double sums[componentsCount][3];
    double prods[componentsCount][3][3];
    int sampleCounts[componentsCount];
    int totalSampleCount;
};


/*
  Calculate beta - parameter of GrabCut algorithm.
  beta = 1/(2*avg(sqr(||color[i] - color[j]||)))
*/
double calcBeta( const cv::Mat& img );
/*
  Calculate weights of noterminal vertices of graph.
  beta and gamma - parameters of GrabCut algorithm.
 */
void calcNWeights( const cv::Mat& img, cv::Mat& leftW, cv::Mat& upleftW, cv::Mat& upW, cv::Mat& uprightW, double beta, double gamma );

/*
  Check size, type and element values of mask matrix.
 */
void checkMask( const cv::Mat& img, const cv::Mat& mask );

/*
  Initialize mask using rectangular.
*/
void initMaskWithRect( cv::Mat& mask, cv::Size imgSize, cv::Rect rect );
/*
  Initialize GMM background and foreground models using kmeans algorithm.
*/
void initGMMs( const cv::Mat& img, const cv::Mat& mask, GMM& bgdGMM, GMM& fgdGMM );
/*
  Assign GMMs components for each pixel.
*/
void assignGMMsComponents( const cv::Mat& img, const cv::Mat& mask, const GMM& bgdGMM, const GMM& fgdGMM, cv::Mat& compIdxs );
/*
  Learn GMMs parameters.
*/
void learnGMMs( const cv::Mat& img, const cv::Mat& mask, const cv::Mat& compIdxs, GMM& bgdGMM, GMM& fgdGMM );
/*
  Construct GCGraph
*/
void constructGCGraph( const cv::Mat& img, const cv::Mat& mask, const GMM& bgdGMM, const GMM& fgdGMM, double lambda,
                       const cv::Mat& leftW, const cv::Mat& upleftW, const cv::Mat& upW, const cv::Mat& uprightW,
                       GCGraph<double>& graph, double bgdFactor );

/*
  Estimate segmentation using MaxFlow algorithm
*/
void estimateSegmentation( GCGraph<double>& graph, cv::Mat& mask );

void cv::grabCut( InputArray _img, InputOutputArray _mask, cv::Rect rect,
                  InputOutputArray _bgdModel, InputOutputArray _fgdModel,
                  int iterCount, int mode );

// MODIFIED

cv::Mat gmmGraphCut(const cv::Mat& img,  cv::Mat bgdModel,cv::Mat fgdModel, double bgdFactor);
