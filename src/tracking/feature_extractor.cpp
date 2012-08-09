#include "feature_extractor.h"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include "simulation/util.h"
#include "utils/conversions.h"
#include "utils/testing.h"
#include <boost/foreach.hpp>
#include "clouds/utils_pcl.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "utils_tracking.h"
#include "utils/testing.h"
#include <algorithm>
#include "utils/utils_vector.h"

using namespace std;
using namespace Eigen;

int FeatureExtractor::m_dim = 0;
vector<FeatureExtractor::FeatureType> FeatureExtractor::m_types = vector<FeatureExtractor::FeatureType>(0);
int FeatureExtractor::m_allDim = 0;
vector<int> FeatureExtractor::m_allSizes = vector<int>(0);
vector<int> FeatureExtractor::m_allStartCols = vector<int>(0);
vector<int> FeatureExtractor::m_sizes = vector<int>(0);
vector<int> FeatureExtractor::m_startCols = vector<int>(0);
vector<FeatureExtractor::FeatureType> FeatureExtractor::m_allType2Ind = vector<FeatureExtractor::FeatureType>(0);

FeatureExtractor::FeatureExtractor() {
	m_types = TrackingConfig::featureTypes;
	m_dim = calcFeatureDim(m_types);
	m_allSizes = vector<int>(FT_SIZES, FT_SIZES+FT_COUNT);
	m_allDim = 0;
	BOOST_FOREACH(int dim, m_allSizes) m_allDim+=dim;
	calcFeatureSubsetIndices(m_types, m_allSizes, m_allStartCols, m_sizes, m_startCols, m_allType2Ind);
}

Eigen::MatrixXf FeatureExtractor::getFeatures(FeatureType fType) {
	if (find(m_types.begin(), m_types.end(), fType) == m_types.end()) return computeFeature(fType);
	return getFeatureCols(fType);
}

int FeatureExtractor::calcFeatureDim(const std::vector<FeatureType>& featureTypes) {
  int dim = 0;
  BOOST_FOREACH(const FeatureType& fType, featureTypes) {
  	dim += FT_SIZES[fType];
  }
  return dim;
}

//  Example:
//	sub_types     [ 1, 3 ]
//	all_sizes     [ 2, 3, 4, 2, 1 ]
//  all_startCols [ 0, 2, 5, 9, 11 ]
//	sub_sizes     [ 3, 2 ]
//	sub_startCols [ 0, 3 ]
//	all2sub       [ -1, 0, -1, 1, -1 ]
void FeatureExtractor::calcFeatureSubsetIndices(const vector<int>& sub_types, const vector<int>& all_sizes, vector<int>&all_startCols, vector<int> &sub_sizes, vector<int>& sub_startCols, vector<int>& all2sub) {
	int nAll = all_sizes.size();
	int nSub = sub_types.size();
	all_startCols = vector<int>(nAll);
	sub_sizes = vector<int>(nSub);
	all2sub = vector<int>(nAll, -1);
	sub_startCols = vector<int>(nSub);
	all_startCols[0] = 0;
	for (int iAll=1; iAll<nAll; iAll++) {
		all_startCols[iAll] = all_startCols[iAll-1] + all_sizes[iAll-1];
	}
	for (int iSub=0; iSub<nSub; iSub++) {
		sub_sizes[iSub] = all_sizes[sub_types[iSub]];
	}
	sub_startCols[0] = 0;
	for (int iSub=1; iSub<nSub; iSub++) {
		sub_startCols[iSub] = sub_startCols[iSub-1] + sub_sizes[iSub-1];
	}
	for (int iSub=0; iSub<nSub; iSub++) {
		all2sub[sub_types[iSub]] = iSub;
	}
}

void CloudFeatureExtractor::updateInputs(ColorCloudPtr cloud) {
	m_cloud = cloud;
	m_features.resize(m_cloud->size(), m_dim);
}

void CloudFeatureExtractor::updateFeatures() {
	//update ALL the features
	BOOST_FOREACH(FeatureType& fType, m_types) {
		setFeatureCols(fType, computeFeature(fType));
	}
}

Eigen::MatrixXf CloudFeatureExtractor::computeFeature(FeatureType fType) {
	Eigen::MatrixXf feature;
	if (fType == FT_XYZ) {
		feature = toEigenMatrix(m_cloud);
	}
	else if (fType == FT_BGR) {
    MatrixXu bgr = toBGR(m_cloud);
    feature = bgr.cast<float>() / 255.;
	}
	else if (fType == FT_LAB) {
    MatrixXu bgr = toBGR(m_cloud);
    cv::Mat cvmat(cv::Size(m_cloud->size(),1), CV_8UC3, bgr.data());
    cv::cvtColor(cvmat, cvmat, CV_BGR2Lab);
    Map<MatrixXu>lab(cvmat.data,m_cloud->size(), 3);
    feature = lab.cast<float>() / 255.;
	}
	else {
		throw std::runtime_error("feature type not yet implemented");
	}
	return feature;
}



TrackedObjectFeatureExtractor::TrackedObjectFeatureExtractor(TrackedObject::Ptr obj) :
	FeatureExtractor(),
	m_obj(obj)
{
	m_features.resize(m_obj->m_nNodes, m_dim);
	//initialize ALL the features
	BOOST_FOREACH(FeatureType& fType, m_types) {
		setFeatureCols(fType, computeFeature(fType));
	}
}

void TrackedObjectFeatureExtractor::updateFeatures() {
	//update only the features that need to be updated
	BOOST_FOREACH(FeatureType& fType, m_types) {
		if (fType == FT_XYZ) {
			setFeatureCols(fType, computeFeature(fType));
		}
	}
}

Eigen::MatrixXf TrackedObjectFeatureExtractor::computeFeature(FeatureType fType) {
	Eigen::MatrixXf feature;
	if (fType == FT_XYZ) {
		feature = toEigenMatrix(m_obj->getPoints());
	}
	else if (fType == FT_BGR) {
		feature = m_obj->getColors();
	}
	else if (fType == FT_LAB) {
		feature = colorTransform(m_obj->getColors(), CV_BGR2Lab);
	}
	else {
		throw std::runtime_error("feature type not yet implemented");
	}
	return feature;
}
