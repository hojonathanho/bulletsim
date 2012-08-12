#pragma once
#include <vector>
#include <Eigen/Dense>
#include <cv.h>
#include "clouds/pcl_typedefs.h"
#include "config_tracking.h"
#include "tracked_object.h"
#include "utils_tracking.h"

class FeatureExtractor {
public:
	typedef boost::shared_ptr<FeatureExtractor> Ptr;
	enum
	{
		FT_ALL = -1,
		FT_XYZ = 0,
		FT_BGR,
		FT_LAB,
		FT_NORMAL,
		FT_LABEL,
		FT_SURF,
		FT_PCASURF,

		FT_COUNT
	};
	typedef int FeatureType;

	//Don't modify them directly
	static std::vector<FeatureType> m_types;
	static int m_dim;
	static int m_allDim;
	static const int FT_SIZES[];

  FeatureExtractor();

  //The children of this class is responsible for updating m_features before this are called
  //returns the submatrix of m_features that contain the feature fType.
  Eigen::Block<Eigen::MatrixXf> getFeatures(FeatureType fType=FT_ALL);

  virtual void updateFeatures() = 0;
  virtual Eigen::MatrixXf computeFeature(FeatureType fType) = 0;

  static Eigen::MatrixXf all2ActiveFeatures(const Eigen::MatrixXf& all_features) {
  	Eigen::MatrixXf active_features(all_features.rows(), m_dim);
  	BOOST_FOREACH(FeatureType& fType, m_types) {
  		active_features.middleCols(m_startCols[m_allType2Ind[fType]], m_sizes[m_allType2Ind[fType]]) = all_features.middleCols(m_allStartCols[fType], m_allSizes[fType]);
  	}
  	return active_features;
  }

  static Eigen::MatrixXf activeFeatures2Feature(const Eigen::MatrixXf active_features, FeatureType fType) {
  	return active_features.middleCols(m_startCols[m_allType2Ind[fType]], m_sizes[m_allType2Ind[fType]]);
  }

protected:
  Eigen::MatrixXf m_features;
  static cv::PCA pca_surf;

private:
	static std::vector<int> m_allSizes;
	static std::vector<int> m_allStartCols;
	static std::vector<int> m_sizes;
	static std::vector<int> m_startCols;
	static std::vector<FeatureType> m_allType2Ind;

  int calcFeatureDim(const std::vector<FeatureType>& featureTypes);
  void calcFeatureSubsetIndices(const vector<int>& sub_types, const vector<int>& all_sizes, vector<int>&all_startCols, vector<int> &sub_sizes, vector<int>& sub_startCols, vector<int>& all2sub);
};

typedef FeatureExtractor FE;

class CloudFeatureExtractor : public FeatureExtractor {
public:
	typedef boost::shared_ptr<CloudFeatureExtractor> Ptr;
	ColorCloudPtr m_cloud;
	cv::Mat m_image;
	CoordinateTransformer* m_transformer;

	void updateInputs(ColorCloudPtr cloud);
	void updateInputs(ColorCloudPtr cloud, cv::Mat image, CoordinateTransformer* transformer);
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
