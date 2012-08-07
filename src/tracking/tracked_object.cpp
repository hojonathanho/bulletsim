#include "tracked_object.h"
#include "utils/conversions.h"
#include "config_tracking.h"
using namespace std;

TrackedObject::TrackedObject(EnvironmentObject::Ptr sim, string type) : m_sim(sim), m_type(type) {
	  setFeatureTypes(TrackingConfig::featureTypes);
}


void TrackedObject::setFeatureTypes(const std::vector<FeatureType>& featureTypes) {
  m_featureDim = calcFeatureDim(featureTypes);
  m_featureTypes = featureTypes;
  m_features.resize(m_nNodes, m_featureDim);
}

Eigen::MatrixXf& TrackedObject::getFeatures() {
  updateFeatures();
  return m_features;
}

Eigen::MatrixXf& TrackedObject::getColors() {
  return m_colors;
}

void TrackedObject::updateFeatures() {
  int col = 0;

  BOOST_FOREACH(FeatureType& ft, m_featureTypes) {
    Eigen::MatrixXf submat;
    bool blockUpdated = false;
    if (ft == FEAT_XYZ) {
      submat = toEigenMatrix(getPoints());      
      blockUpdated = true;
    }
    else if (ft == FEAT_LAB) {
      // note: we only have to do this the first time
      assert(getColors().cols() == 3);
      MatrixXu bgr = (getColors()*255).cast<uint8_t>();
      cv::Mat cvmat(cv::Size(m_nNodes,1), CV_8UC3, bgr.data());
      cv::cvtColor(cvmat, cvmat, CV_BGR2Lab);
      Eigen::Map<MatrixXu>lab(cvmat.data,m_nNodes, 3);
      submat = lab.cast<float>() / 255.; 
      blockUpdated = true;
    }
    else {
      throw std::runtime_error("feature type not yet implemented");
    }
    
    if (blockUpdated) m_features.middleCols(col,FEATURE_SIZES[FEAT_XYZ]) = submat;
    col += FEATURE_SIZES[ft];        
    
  }
}



std::vector<btVector3> calcImpulsesDamped(const std::vector<btVector3>& estPos, const std::vector<btVector3>& estVel, 
  const std::vector<btVector3>& obsPts, const SparseMatrixf& corr, const vector<float>& masses, float kp, float kd) {
  int nEst = estPos.size();
  int nObs = obsPts.size();
  vector<btVector3> out(nEst);

  for (int iEst=0; iEst<corr.rows(); ++iEst) {
    btVector3 dv = -kd * estVel[iEst];
    for (SparseMatrixf::InnerIterator it(corr,iEst); it; ++it)
    {
      dv += (kp * it.value()) * (obsPts[it.col()] - estPos[iEst]);
    } 
    out[iEst] = masses[iEst]*dv;
    // XXX SHOULD THERE BE DT?
  }

  return out;
}
