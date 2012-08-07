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

using namespace std;
using namespace Eigen;

FeatureExtractor::FeatureExtractor() :
	m_featureTypes(TrackingConfig::featureTypes),
	m_featureDim(calcFeatureDim(m_featureTypes)),
	m_featureStartCol(calcFeatureStartCol(m_featureTypes))
{}

Eigen::MatrixXf FeatureExtractor::getFeatures(FeatureType feature_type) {
	if (feature_type <0 || feature_type >= FT_COUNT)
		throw std::runtime_error("feature type not yet implemented");
	if (m_featureStartCol[feature_type] < 0)
		throw std::runtime_error("feature is not being used");
	return m_features.middleCols(m_featureStartCol[feature_type], FT_SIZES[feature_type]);
}

int FeatureExtractor::calcFeatureDim(const std::vector<FeatureType>& featureTypes) {
  int dim = 0;
  BOOST_FOREACH(const FeatureType& ft, featureTypes) dim += FT_SIZES[ft];
  return dim;
}

std::vector<int> FeatureExtractor::calcFeatureStartCol(const std::vector<FeatureType>& featureTypes) {
	vector<int> startCol(FeatureExtractor::FT_COUNT, -1);
	startCol[featureTypes[0]] = 0;
	for (int featInd=1; featInd<featureTypes.size(); featInd++) {
		startCol[featureTypes[featInd]] = startCol[featureTypes[featInd-1]] + FT_SIZES[featureTypes[featInd-1]];
	}
	return startCol;
}

void CloudFeatureExtractor::updateFeatures(ColorCloudPtr cloud) {
	m_features.resize(cloud->size(), m_featureDim);
  int col = 0;
  //update ALL the features
  BOOST_FOREACH(FeatureType& ft, m_featureTypes) {
    MatrixXf submat;
		if (ft == FT_XYZ) {
      submat = toEigenMatrix(cloud);
    }
    else if (ft == FT_LAB) {
      MatrixXu bgr = toBGR(cloud);
      cv::Mat cvmat(cv::Size(cloud->size(),1), CV_8UC3, bgr.data());
      cv::cvtColor(cvmat, cvmat, CV_BGR2Lab);
      Map<MatrixXu>lab(cvmat.data,cloud->size(), 3);
      submat = lab.cast<float>() / 255.;
    }
		else {
			if (!(ft >= 0 || ft < FT_COUNT))
				throw std::runtime_error("feature type not yet implemented");
		}

    m_features.middleCols(col,FT_SIZES[FT_XYZ]) = submat;
    col += FT_SIZES[ft];
  }
}

TrackedObjectFeatureExtractor::TrackedObjectFeatureExtractor(TrackedObject::Ptr obj) :
	FeatureExtractor(),
	m_obj(obj)
{
	m_features.resize(m_obj->m_nNodes, m_featureDim);
	int col = 0;
	//initialize ALL the features
	BOOST_FOREACH(FeatureType& ft, m_featureTypes) {
		Eigen::MatrixXf submat;
		if (ft == FT_XYZ) {
			submat = toEigenMatrix(m_obj->getPoints());
		}
		else if (ft == FT_BGR) {
			submat = m_obj->getColors();
		}
		else if (ft == FT_LAB) {
			submat = colorTransform(m_obj->getColors(), CV_BGR2Lab);
		}
		else {
			throw std::runtime_error("feature type not yet implemented");
		}

		m_features.middleCols(col,FT_SIZES[ft]) = submat;
		col += FT_SIZES[ft];
	}
}

void TrackedObjectFeatureExtractor::updateFeatures() {
	int col = 0;
	//update only the features that need to
	BOOST_FOREACH(FeatureType& ft, m_featureTypes) {
		Eigen::MatrixXf submat;
		if (ft == FT_XYZ) {
			submat = toEigenMatrix(m_obj->getPoints());
		}
		else if (ft >= 0 || ft < FT_COUNT) {
			col += FT_SIZES[ft];
			continue;
		}
		else {
			throw std::runtime_error("feature type not yet implemented");
		}

		m_features.middleCols(col,FT_SIZES[ft]) = submat;
		col += FT_SIZES[ft];
	}
}
