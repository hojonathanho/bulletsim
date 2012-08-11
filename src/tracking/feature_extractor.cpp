#include "feature_extractor.h"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include "simulation/util.h"
#include "utils/conversions.h"
#include "utils/testing.h"
#include <boost/foreach.hpp>
#include "clouds/utils_pcl.h"
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "utils/testing.h"
#include <algorithm>
#include "utils/utils_vector.h"
#include "utils/cvmat.h"

using namespace std;
using namespace Eigen;

//Modify this if you change the FeatureTypes
const int FeatureExtractor::FT_SIZES[] = { 3, 3, 3, 3, 1, 64, 2 };

int FeatureExtractor::m_dim = 0;
vector<FeatureExtractor::FeatureType> FeatureExtractor::m_types = vector<FeatureExtractor::FeatureType> (0);
int FeatureExtractor::m_allDim = 0;
vector<int> FeatureExtractor::m_allSizes = vector<int> (0);
vector<int> FeatureExtractor::m_allStartCols = vector<int> (0);
vector<int> FeatureExtractor::m_sizes = vector<int> (0);
vector<int> FeatureExtractor::m_startCols = vector<int> (0);
vector<FeatureExtractor::FeatureType> FeatureExtractor::m_allType2Ind = vector<FeatureExtractor::FeatureType> (0);

cv::PCA FE::pca_surf = cv::PCA();

FeatureExtractor::FeatureExtractor() {
	if (sizeof(FT_SIZES) / sizeof(const int) != FT_COUNT)
		throw runtime_error("The number of FeatureType doesn't match the size of the FT_SIZES array.");

	m_types = TrackingConfig::featureTypes;
	m_dim = calcFeatureDim(m_types);
	m_allSizes = vector<int> (FT_SIZES, FT_SIZES + FT_COUNT);
	m_allDim = 0;
	BOOST_FOREACH(int dim, m_allSizes) m_allDim	+=dim;
	calcFeatureSubsetIndices(m_types, m_allSizes, m_allStartCols, m_sizes, m_startCols, m_allType2Ind);
}

Eigen::MatrixXf FeatureExtractor::getFeatures(FeatureType fType) {
	if (find(m_types.begin(), m_types.end(), fType) == m_types.end())
		return computeFeature(fType);
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
void FeatureExtractor::calcFeatureSubsetIndices(const vector<int>& sub_types,
		const vector<int>& all_sizes, vector<int>&all_startCols,
		vector<int> &sub_sizes, vector<int>& sub_startCols, vector<int>& all2sub) {
	int nAll = all_sizes.size();
	int nSub = sub_types.size();
	all_startCols = vector<int> (nAll);
	sub_sizes = vector<int> (nSub);
	all2sub = vector<int> (nAll, -1);
	sub_startCols = vector<int> (nSub);
	all_startCols[0] = 0;
	for (int iAll = 1; iAll < nAll; iAll++) {
		all_startCols[iAll] = all_startCols[iAll - 1] + all_sizes[iAll - 1];
	}
	for (int iSub = 0; iSub < nSub; iSub++) {
		sub_sizes[iSub] = all_sizes[sub_types[iSub]];
	}
	sub_startCols[0] = 0;
	for (int iSub = 1; iSub < nSub; iSub++) {
		sub_startCols[iSub] = sub_startCols[iSub - 1] + sub_sizes[iSub - 1];
	}
	for (int iSub = 0; iSub < nSub; iSub++) {
		all2sub[sub_types[iSub]] = iSub;
	}
}


void CloudFeatureExtractor::updateInputs(ColorCloudPtr cloud) {
	m_cloud = cloud;
	m_features.resize(m_cloud->size(), m_dim);
}

void CloudFeatureExtractor::updateInputs(ColorCloudPtr cloud, cv::Mat image, CoordinateTransformer* transformer) {
	m_cloud = cloud;
	m_image = image;
	m_transformer = transformer;
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
	} else if (fType == FT_BGR) {
		MatrixXu bgr = toBGR(m_cloud);
		feature = bgr.cast<float> () / 255.;
	} else if (fType == FT_LAB) {
		MatrixXu bgr = toBGR(m_cloud);
		cv::Mat cvmat(cv::Size(m_cloud->size(), 1), CV_8UC3, bgr.data());
		cv::cvtColor(cvmat, cvmat, CV_BGR2Lab);
		Map<MatrixXu> lab(cvmat.data, m_cloud->size(), 3);
		feature = lab.cast<float> () / 255.;
	} else if (fType == FT_SURF) {
		feature.resize(m_cloud->size(), FT_SIZES[FT_SURF]);
	  MatrixXf ptsCam = toEigenMatrix(m_transformer->toCamFromWorldN(toBulletVectors(m_cloud)));
	  MatrixXi uvs = xyz2uv(ptsCam);

	  vector<cv::KeyPoint> keypoints(m_cloud->size());
		for (int i = 0; i < m_cloud->size(); i++) {
			keypoints[i] = cv::KeyPoint(uvs(i,1), uvs(i,0), TrackingConfig::node_pixel);
		}
		cv::SURF surf(400, 4, 2, false);
		cv::Mat mask = cv::Mat::ones(m_image.rows, m_image.cols, CV_8UC1);
		vector<float> descriptors;
		surf(m_image, mask, keypoints, descriptors, true);
		assert(surf.descriptorSize() == FT_SIZES[FT_SURF]);
		for (int i = 0; i < m_cloud->size(); i++) {
			for (int descInd = 0; descInd < surf.descriptorSize(); descInd++) {
				feature(i, descInd) = descriptors[i * surf.descriptorSize() + descInd];
			}
		}
		cv::Mat keypoints_image = m_image.clone();
		for(int i=0; i<keypoints.size(); i++) {
			cv::circle(keypoints_image, keypoints[i].pt, keypoints[i].size, cv::Scalar(0,255,0), 1, 8, 0);
		}
		cv::imwrite("/home/alex/Desktop/keypoints_cloud.jpg", keypoints_image);

	} else if (fType == FT_PCASURF) {
		MatrixXf surf_feature = computeFeature(FT_SURF);
		if (pca_surf.mean.empty()) {
			cv::Mat cv_surf_feature(surf_feature.rows(), surf_feature.cols(), CV_32FC1, surf_feature.data());
			pca_surf = cv::PCA(cv_surf_feature, cv::Mat(), CV_PCA_DATA_AS_ROW, FT_SIZES[FT_PCASURF]);
		}
		feature = compressPCA(pca_surf, surf_feature);

	} else {
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
	} else if (fType == FT_BGR) {
		feature = m_obj->getColors();
	} else if (fType == FT_LAB) {
		feature = colorTransform(m_obj->getColors(), CV_BGR2Lab);
	} else if (fType == FT_SURF) {
		feature.resize(m_obj->m_nNodes, FT_SIZES[FT_SURF]);
		TrackedTowel* tracked_towel = dynamic_cast<TrackedTowel*> (m_obj.get());
		cv::Mat tex_image = tracked_towel->getSim()->getTexture();

		if (tex_image.empty())
			throw std::runtime_error("unable to make SURF features without any texture image");

		const osg::Vec2Array& texcoords = *(tracked_towel->getSim()->tritexcoords);
		vector<cv::KeyPoint> keypoints(m_obj->m_nNodes);
		for (int i = 0; i < m_obj->m_nNodes; i++) {
			cv::Point2f uv = tracked_towel->textureCoordinate(i);
			keypoints[i] = cv::KeyPoint(uv.y, uv.x, TrackingConfig::node_pixel);
		}
		cv::SURF surf(400, 4, 2, false);
		cv::Mat mask = cv::Mat::ones(tex_image.rows, tex_image.cols, CV_8UC1);
		vector<float> descriptors;
		surf(tex_image, mask, keypoints, descriptors, true);
		assert(surf.descriptorSize() == FT_SIZES[FT_SURF]);
		for (int i = 0; i < m_obj->m_nNodes; i++) {
			for (int descInd = 0; descInd < surf.descriptorSize(); descInd++) {
				feature(i, descInd) = descriptors[i * surf.descriptorSize() + descInd];
			}
		}
		cv::Mat keypoints_image = tex_image.clone();
		for(int i=0; i<keypoints.size(); i++) {
			cv::circle(keypoints_image, keypoints[i].pt, keypoints[i].size, cv::Scalar(0,255,0), 1, 8, 0);
		}
		cv::imwrite("/home/alex/Desktop/keypoints_obj.jpg", keypoints_image);

	} else if (fType == FT_PCASURF) {
		MatrixXf surf_feature = computeFeature(FT_SURF);
		if (pca_surf.mean.empty()) {
			cv::Mat cv_surf_feature(surf_feature.rows(), surf_feature.cols(), CV_32FC1, surf_feature.data());
			pca_surf = cv::PCA(cv_surf_feature, cv::Mat(), CV_PCA_DATA_AS_ROW, FT_SIZES[FT_PCASURF]);
		}
		feature = compressPCA(pca_surf, surf_feature);

	} else {
		throw std::runtime_error("feature type not yet implemented");
	}

	assert(feature.rows() == m_obj->m_nNodes);
	assert(feature.cols() == FT_SIZES[fType]);
	return feature;
}
