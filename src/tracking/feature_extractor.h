#pragma once
#include <vector>
#include <Eigen/Dense>
#include "sparse_utils.h"
#include "clouds/pcl_typedefs.h"
#include "config_tracking.h"
#include "tracked_object.h"
#include "tracking_defs.h"

class FeatureExtractor {
public:
	typedef boost::shared_ptr<FeatureExtractor> Ptr;
	enum
	{
		FT_XYZ = 0,
		FT_BGR,
		FT_LAB,
		FT_NORMAL,
		FT_LABEL,

		FT_COUNT
	};
	typedef int FeatureType;

	int m_dim;
	std::vector<int> m_types;
	std::vector<int> m_allSizes;
	std::vector<int> m_allStartCols;
	std::vector<int> m_sizes;
  std::vector<int> m_startCols;
  std::vector<int> m_allType2Ind;

  FeatureExtractor();

  //The children of this class is responsible for updating m_features before these are called
  Eigen::MatrixXf& getFeatures() { return m_features; }
  Eigen::MatrixXf getFeatures(FeatureType fType);

  Eigen::MatrixXf all2ActiveFeatures(const Eigen::MatrixXf& all_features) {
  	Eigen::MatrixXf active_features(all_features.rows(), m_dim);
  	BOOST_FOREACH(FeatureType& fType, m_types) {
  		active_features.middleCols(m_startCols[m_allType2Ind[fType]], m_sizes[m_allType2Ind[fType]]) = all_features.middleCols(m_allStartCols[fType], m_allSizes[fType]);
  	}
  }

  virtual void updateFeatures() = 0;
  virtual Eigen::MatrixXf computeFeature(FeatureType fType) = 0;

protected:
  Eigen::MatrixXf m_features;

  //Don't touch these
  inline Eigen::MatrixXf getFeatureCols(FeatureType fType) {
  	return m_features.middleCols(m_startCols[m_allType2Ind[fType]], m_sizes[m_allType2Ind[fType]]);
  }
  void setFeatureCols(FeatureType fType, const Eigen::MatrixXf& fCols) {
  	assert(m_sizes[m_allType2Ind[fType]] == fCols.cols());
  	m_features.middleCols(m_startCols[m_allType2Ind[fType]], m_sizes[m_allType2Ind[fType]]) = fCols;
  }

private:
  int calcFeatureDim(const std::vector<FeatureType>& featureTypes);
  void calcFeatureSubsetIndices(const vector<int>& sub_types, const vector<int>& all_sizes, vector<int>&all_startCols, vector<int> &sub_sizes, vector<int>& sub_startCols, vector<int>& all2sub);
};


class CloudFeatureExtractor : public FeatureExtractor {
public:
	typedef boost::shared_ptr<CloudFeatureExtractor> Ptr;
	ColorCloudPtr m_cloud;

	void updateInputs(ColorCloudPtr cloud);
	void updateFeatures();
  Eigen::MatrixXf computeFeature(FeatureType fType);
};


class TrackedObjectFeatureExtractor : public FeatureExtractor {
public:
	typedef boost::shared_ptr<TrackedObjectFeatureExtractor> Ptr;
	TrackedObject::Ptr m_obj;

  TrackedObjectFeatureExtractor(TrackedObject::Ptr obj);
  void updateFeatures();
  Eigen::MatrixXf computeFeature(FeatureType fType);
};
