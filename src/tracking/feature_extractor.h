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

  std::vector<FeatureType> m_featureTypes;
  int m_featureDim;
  std::vector<int> m_featureStartCol;
  Eigen::MatrixXf m_features;

  FeatureExtractor();

  //The children of this class is responsible for updating m_features before these are called
  Eigen::MatrixXf& getFeatures() { return m_features; }
  Eigen::MatrixXf getFeatures(FeatureType feature_type);

  int calcFeatureDim(const std::vector<FeatureType>& featureTypes);
  std::vector<int> calcFeatureStartCol(const std::vector<FeatureType>& featureTypes);
};

class CloudFeatureExtractor : public FeatureExtractor {
public:
	typedef boost::shared_ptr<CloudFeatureExtractor> Ptr;
  void updateFeatures(ColorCloudPtr cloud);
};

class TrackedObjectFeatureExtractor : public FeatureExtractor {
public:
	typedef boost::shared_ptr<TrackedObjectFeatureExtractor> Ptr;
	TrackedObject::Ptr m_obj;

  TrackedObjectFeatureExtractor(TrackedObject::Ptr obj);

  void updateFeatures();
};
