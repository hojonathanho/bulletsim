#include "simple_physics_tracker.h"
#include "sparse_utils.h"
#include "config_tracking.h"
#include "algorithm_common.h"
#include "plotting_tracking.h"
#include "utils/conversions.h"
#include "utils/clock.h"
#include <fstream>
#include <boost/thread.hpp>
#include "utils/testing.h"
#include "feature_extractor.h"

using namespace Eigen;
using namespace std;

//#define CHECK_CORRECTNESS

SimplePhysicsTracker::SimplePhysicsTracker(TrackedObject::Ptr obj, VisibilityInterface::Ptr visInt, Environment::Ptr env) :
  m_obj(obj),
  m_visInt(visInt),
  m_env(env),
  m_obsCloud(new ColorCloud()),
	m_obsInlierPlot(new PointCloudPlot(3)),
	m_obsPlot(new PointCloudPlot(6)),
	m_obsTransPlot(new PointCloudPlot(6)),
	m_estPlot(new PlotSpheres()),
	m_estTransPlot(new PlotSpheres()),
	m_corrPlot(new PlotLines()),
	m_debugPlot(new PointCloudPlot(6)),
	m_enableObsInlierPlot(false),
	m_enableObsPlot(false),
	m_enableObsTransPlot(false),
	m_enableEstPlot(false),
	m_enableEstTransPlot(false),
	m_enableCorrPlot(false),
	m_enableDebugPlot(false),
	m_applyEvidence(true),
	m_count(-1)
{
	m_env->add(m_obsInlierPlot);
	m_env->add(m_obsPlot);
	m_env->add(m_obsTransPlot);
	m_env->add(m_estPlot);
	m_env->add(m_estTransPlot);
	m_env->add(m_corrPlot);
	m_env->add(m_debugPlot);
	m_corrPlot->setDefaultColor(1,1,0,.3);

	m_obj_features = TrackedObjectFeatureExtractor::Ptr(new TrackedObjectFeatureExtractor(m_obj));
	m_cloud_features = CloudFeatureExtractor::Ptr(new CloudFeatureExtractor());

	m_prior_dist = m_obj->getPriorDist();
	m_stdev = m_prior_dist.transpose().replicate(m_obj->m_nNodes, 1);

	m_outlier_dist = m_obj->getOutlierDist();
}

void SimplePhysicsTracker::updateInput(ColorCloudPtr obsPts) {
	m_cloud_features->updateInputs(obsPts);

	m_obj_features->updateFeatures();
	m_cloud_features->updateFeatures();

	m_obsPts = m_cloud_features->getFeatures();
	//m_obsPts.col(2) = m_obsPts.col(2).array() + 0.05*METERS;
	for (int i=0; i<m_obsPts.rows(); i++) {
		if (m_obsPts(i,2) < 0.01*METERS) m_obsPts(i,2) = 0.01*METERS;
	}
	m_obsCloud = obsPts;
}

void SimplePhysicsTracker::doIteration() {
  VectorXf vis = m_visInt->checkNodeVisibility(m_obj);
  m_estPts = m_obj_features->getFeatures();

  // E STEP
  //boost::posix_time::ptime e_time = boost::posix_time::microsec_clock::local_time();
  MatrixXf pZgivenC = calculateResponsibilitiesNaive(m_estPts, m_obsPts, m_stdev, vis, m_obj->getOutlierDist(), m_obj->getOutlierStdev());
  //cout << "E time " << (boost::posix_time::microsec_clock::local_time() - e_time).total_milliseconds() << endl;

#ifdef CHECK_CORRECTNESS
  boost::posix_time::ptime en_time = boost::posix_time::microsec_clock::local_time();
  MatrixXf pZgivenC_naive = calculateResponsibilitiesNaive(m_estPts, m_obsPts, m_stdev, vis, m_outlier_dist, m_prior_dist);
  cout << "E naive time " << (boost::posix_time::microsec_clock::local_time() - en_time).total_milliseconds() << endl;
  assert(isApproxEq(pZgivenC_naive, pZgivenC));
#endif

  VectorXf inlierFrac = pZgivenC.colwise().sum();
  if (m_enableObsInlierPlot) plotObs(toBulletVectors(m_obsPts.leftCols(3)), inlierFrac, m_obsInlierPlot);
	else m_obsInlierPlot->clear();
	if (m_enableObsPlot) plotObs(m_cloud_features->getFeatures(FeatureExtractor::FT_XYZ), m_cloud_features->getFeatures(FeatureExtractor::FT_BGR), m_obsPlot);
	else m_obsPlot->clear();
	if (m_enableObsTransPlot) plotObs(m_cloud_features->getFeatures(FeatureExtractor::FT_XYZ), m_cloud_features->getFeatures(FeatureExtractor::FT_LAB), m_obsTransPlot);
	else m_obsTransPlot->clear();
	if (m_enableEstPlot) plotNodesAsSpheres(toEigenMatrix(m_obj->getPoints()), m_obj->getColors(), vis, m_stdev, m_estPlot);
	else m_estPlot->clear();
	//if (m_enableEstTransPlot) plotNodesAsSpheres(toEigenMatrix(m_obj->getPoints()), m_obj_features->getFeatures(FeatureExtractor::FT_LAB), vis, m_stdev, m_estTransPlot);
	MatrixXf nodes_naive = calculateNodesNaive(m_estPts, m_obsPts, pZgivenC);
	//MatrixXf nodes = calculateNodes(m_estPts, m_obsPts, pZgivenC);
	//assert(isApproxEq(nodes_naive, nodes));
	if (m_enableEstTransPlot) plotNodesAsSpheres(nodes_naive.leftCols(3), nodes_naive.middleCols(3,3), vis, m_stdev, m_estTransPlot);
	else m_estTransPlot->clear();
	if (m_enableCorrPlot) drawCorrLines(m_corrPlot, toBulletVectors(m_obj_features->getFeatures(FeatureExtractor::FT_XYZ)), toBulletVectors(m_obsPts.leftCols(3)), pZgivenC, 0.01, m_count);
	else m_corrPlot->clear();
  if (m_enableDebugPlot) plotObs(m_obsDebug, m_debugPlot);
	else m_debugPlot->clear();

  // M STEP
  //boost::posix_time::ptime evidence_time = boost::posix_time::microsec_clock::local_time();
  if (m_applyEvidence) m_obj->applyEvidence(pZgivenC, m_obsPts);
  //cout << "Evidence time " << (boost::posix_time::microsec_clock::local_time() - evidence_time).total_milliseconds() << endl;

  //boost::posix_time::ptime m_time = boost::posix_time::microsec_clock::local_time();
  m_stdev = calculateStdev(m_estPts, m_obsPts, pZgivenC, m_prior_dist, 10);
  //cout << "M time " << (boost::posix_time::microsec_clock::local_time() - m_time).total_milliseconds() << endl;

#ifdef CHECK_CORRECTNESS
  boost::posix_time::ptime mn_time = boost::posix_time::microsec_clock::local_time();
  MatrixXf stdev_naive = calculateStdevNaive(m_estPts, m_obsPts, pZgivenC, m_prior_dist, 10);
  cout << "M naive time " << (boost::posix_time::microsec_clock::local_time() - mn_time).total_milliseconds() << endl;
  assert(isApproxEq(stdev_naive, m_stdev));
#endif

  m_env->step(.03,2,.015);
}
