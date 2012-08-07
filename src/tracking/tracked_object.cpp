#include "tracked_object.h"
#include "utils/conversions.h"
#include "config_tracking.h"
#include "utils/utils_vector.h"

using namespace std;

TrackedObject::TrackedObject(EnvironmentObject::Ptr sim, string type) : m_sim(sim), m_type(type) { }

void TrackedObject::init() {
	initColors();
	initFeatures();
}

Eigen::MatrixXf& TrackedObject::getFeatures() {
  updateFeatures();
  return m_features;
}

Eigen::MatrixXf TrackedObject::getFeatures(FeatureType feature_type) {
  updateFeatures(feature_type);
  if (feature_type == FEAT_ALL)
  	return m_features;
 	return m_features.middleCols(m_featureStartCol[feature_type], FEATURE_SIZES[feature_type]);
}

// updates only the features that are not constants
void TrackedObject::updateFeatures(FeatureType feature_type) {
	Eigen::MatrixXf submat;

  switch (feature_type) {
		case FEAT_XYZ:
		{
			submat = toEigenMatrix(getPoints());
			break;
		}

		case FEAT_ALL:
		{
			BOOST_FOREACH(FeatureType& ft, m_featureTypes) updateFeatures(ft);
			return;
			break;
		}

		default:
		{
			if (feature_type < FEAT_ALL || feature_type >= FEAT_COUNT)
				throw std::runtime_error("feature type not yet implemented");
			return;
			break;
		}
	}

  m_features.middleCols(m_featureStartCol[feature_type], FEATURE_SIZES[feature_type]) = submat;
}

// computes only the features that are constants (i.e. colors, descriptors, etc.)
void TrackedObject::initFeatures(FeatureType feature_type) {
	Eigen::MatrixXf submat;

  switch (feature_type) {
		case FEAT_BGR:
		{
			submat = getColors();
			break;
		}

		case FEAT_LAB:
		{
			assert(getColors().cols() == 3);
			MatrixXu bgr = (getColors()*255).cast<uint8_t>();
			cv::Mat cvmat(cv::Size(m_nNodes,1), CV_8UC3, bgr.data());
			cv::cvtColor(cvmat, cvmat, CV_BGR2Lab);
			Eigen::Map<MatrixXu>lab(cvmat.data,m_nNodes, 3);
			submat = lab.cast<float>() / 255.;
			submat = colorTransform(getColors(), CV_BGR2Lab);
			break;
		}

		case FEAT_ALL:
		{
			m_featureTypes = TrackingConfig::featureTypes;
			m_featureDim = calcFeatureDim(m_featureTypes);
			m_featureStartCol = calcFeatureStartCol(m_featureTypes);
			m_features.resize(m_nNodes, m_featureDim);
			BOOST_FOREACH(FeatureType& ft, m_featureTypes) initFeatures(ft);
			return;
			break;
		}

		default:
		{
			if (feature_type < FEAT_ALL || feature_type >= FEAT_COUNT)
				throw std::runtime_error("feature type not yet implemented");
			return;
			break;
		}
	}

  m_features.middleCols(m_featureStartCol[feature_type], FEATURE_SIZES[feature_type]) = submat;
}

Eigen::MatrixXf& TrackedObject::getColors() {
	return m_colors;
}

std::vector<btVector3> calcImpulsesDamped(const std::vector<btVector3>& estPos, const std::vector<btVector3>& estVel,
  const std::vector<btVector3>& obsPts, const Eigen::MatrixXf& corr, const vector<float>& masses, float kp, float kd) {
  int K = estPos.size();
  int N = obsPts.size();
  assert(estVel.size() == K);
  assert(corr.rows() == K);
  assert(corr.cols() == N);
  assert(masses.size() == K);
  vector<btVector3> impulses(K);

  for (int k=0; k<K; k++) {
  	btVector3 dv = -kd * estVel[k];
  	for (int n=0; n<N; n++)
			dv += (kp * corr(k,n)) * (obsPts[n] - estPos[k]);
		impulses[k] = masses[k]*dv;
		// XXX SHOULD THERE BE DT?
  }
  return impulses;
}
