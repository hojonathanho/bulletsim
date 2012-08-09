#include "physics_tracker.h"
#include "config_tracking.h"
#include "algorithm_common.h"
#include "plotting_tracking.h"
#include "utils/conversions.h"
#include "utils_tracking.h"
#include <boost/thread.hpp>
#include "utils/testing.h"
#include "feature_extractor.h"
#include "utils/logging.h"
#include <cv.h>

using namespace Eigen;
using namespace std;

//#define CHECK_CORRECTNESS

PhysicsTracker::PhysicsTracker(TrackedObjectFeatureExtractor::Ptr object_features, FeatureExtractor::Ptr observation_features, VisibilityInterface::Ptr visibility_interface) :
	m_objFeatures(object_features),
	m_obsFeatures(observation_features),
	m_visInt(visibility_interface)
{
	m_priorDist = m_objFeatures->m_obj->getPriorDist();
	m_stdev = m_priorDist.transpose().replicate(m_objFeatures->m_obj->m_nNodes, 1);

	m_outlierDist = m_objFeatures->m_obj->getOutlierDist();
}

// Before calling this function, the inputs of the FeatureExtractors should be updated (if any)
void PhysicsTracker::updateFeatures() {
	m_objFeatures->updateFeatures();
	m_obsFeatures->updateFeatures();

	m_estPts = m_objFeatures->getFeatures();
	m_obsPts = m_obsFeatures->getFeatures();
	//m_obsPts.col(2) = m_obsPts.col(2).array() + 0.05*METERS;
	for (int i=0; i<m_obsPts.rows(); i++) {
		if (m_obsPts(i,2) < 0.01*METERS) m_obsPts(i,2) = 0.01*METERS;
	}

	m_vis = m_visInt->checkNodeVisibility(m_objFeatures->m_obj);
}

void PhysicsTracker::expectationStep() {
  boost::posix_time::ptime e_time = boost::posix_time::microsec_clock::local_time();
  m_pZgivenC = calculateResponsibilities(m_estPts, m_obsPts, m_stdev, m_vis, m_objFeatures->m_obj->getOutlierDist(), m_objFeatures->m_obj->getOutlierStdev());
  LOG_DEBUG("E time " << (boost::posix_time::microsec_clock::local_time() - e_time).total_milliseconds());

#ifdef CHECK_CORRECTNESS
  boost::posix_time::ptime en_time = boost::posix_time::microsec_clock::local_time();
  MatrixXf pZgivenC_naive = calculateResponsibilitiesNaive(m_estPts, m_obsPts, m_stdev, m_vis, m_objFeatures->m_obj->getOutlierDist(), m_objFeatures->m_obj->getOutlierStdev());
  cout << "E naive time " << (boost::posix_time::microsec_clock::local_time() - en_time).total_milliseconds() << endl;
  assert(isApproxEq(pZgivenC_naive, m_pZgivenC));
#endif
}

void PhysicsTracker::maximizationStep(bool apply_evidence) {
  boost::posix_time::ptime evidence_time = boost::posix_time::microsec_clock::local_time();
  if (apply_evidence) m_objFeatures->m_obj->applyEvidence(m_pZgivenC, m_obsPts);
  LOG_DEBUG("Evidence time " << (boost::posix_time::microsec_clock::local_time() - evidence_time).total_milliseconds());

  boost::posix_time::ptime m_time = boost::posix_time::microsec_clock::local_time();
  m_stdev = calculateStdev(m_estPts, m_obsPts, m_pZgivenC, m_priorDist, 10);
  LOG_DEBUG("M time " << (boost::posix_time::microsec_clock::local_time() - m_time).total_milliseconds());

#ifdef CHECK_CORRECTNESS
  boost::posix_time::ptime mn_time = boost::posix_time::microsec_clock::local_time();
  MatrixXf stdev_naive = calculateStdevNaive(m_estPts, m_obsPts, m_pZgivenC, m_priorDist, 10);
  cout << "M naive time " << (boost::posix_time::microsec_clock::local_time() - mn_time).total_milliseconds() << endl;
  assert(isApproxEq(stdev_naive, m_stdev));
#endif
}


PhysicsTrackerVisualizer::PhysicsTrackerVisualizer(Scene* scene, PhysicsTracker::Ptr tracker) :
	m_scene(scene),
	m_tracker(tracker),

	m_obsInlierPlot(new PointCloudPlot(3)),
	m_obsPlot(new PointCloudPlot(6)),
	m_obsTransPlot(new PointCloudPlot(6)),
	m_estPlot(new PlotSpheres()),
	m_estTransPlot(new PlotSpheres()),
	m_corrPlot(new PlotLines()),

	m_enableObsInlierPlot(false),
	m_enableObsPlot(false),
	m_enableObsTransPlot(false),
	m_enableEstPlot(false),
	m_enableEstTransPlot(false),
	m_enableCorrPlot(false),
	m_nodeCorrPlot(-1)
{
	m_scene->env->add(m_obsInlierPlot);
	m_scene->env->add(m_obsPlot);
	m_scene->env->add(m_obsTransPlot);
	m_scene->env->add(m_estPlot);
	m_scene->env->add(m_estTransPlot);
	m_scene->env->add(m_corrPlot);
	m_corrPlot->setDefaultColor(1,1,0,.3);

	m_scene->addVoidKeyCallback('c',boost::bind(toggle, &m_enableCorrPlot));
	m_scene->addVoidKeyCallback('C',boost::bind(toggle, &m_enableCorrPlot));
	m_scene->addVoidKeyCallback('e',boost::bind(toggle, &m_enableEstPlot));
	m_scene->addVoidKeyCallback('E',boost::bind(toggle, &m_enableEstTransPlot));
	m_scene->addVoidKeyCallback('o',boost::bind(toggle, &m_enableObsPlot));
	m_scene->addVoidKeyCallback('O',boost::bind(toggle, &m_enableObsTransPlot));
	m_scene->addVoidKeyCallback('i',boost::bind(toggle, &m_enableObsInlierPlot));
	m_scene->addVoidKeyCallback('I',boost::bind(toggle, &m_enableObsInlierPlot));

  m_scene->addVoidKeyCallback('[',boost::bind(add, &m_nodeCorrPlot, -1));
  m_scene->addVoidKeyCallback(']',boost::bind(add, &m_nodeCorrPlot, 1));
}

void PhysicsTrackerVisualizer::update() {
  MatrixXf& estPts = m_tracker->m_pZgivenC;
  MatrixXf& stdev = m_tracker->m_stdev;
  MatrixXf& obsPts = m_tracker->m_obsPts;
  MatrixXf& pZgivenC = m_tracker->m_pZgivenC;
  VectorXf& vis = m_tracker->m_vis;

  FeatureExtractor::Ptr obsFeatures = m_tracker->m_obsFeatures;
  TrackedObjectFeatureExtractor::Ptr objFeatures = m_tracker->m_objFeatures;
  TrackedObject::Ptr obj = objFeatures->m_obj;

	if (m_enableObsInlierPlot) plotObs(toBulletVectors(obsPts.leftCols(3)), pZgivenC.colwise().sum(), m_obsInlierPlot);
	else m_obsInlierPlot->clear();

	if (m_enableObsPlot) plotObs(obsFeatures->getFeatures(FeatureExtractor::FT_XYZ), obsFeatures->getFeatures(FeatureExtractor::FT_BGR), m_obsPlot);
	else m_obsPlot->clear();
	if (m_enableObsTransPlot) plotObs(obsFeatures->getFeatures(FeatureExtractor::FT_XYZ), obsFeatures->getFeatures(FeatureExtractor::FT_LAB), m_obsTransPlot);
	else m_obsTransPlot->clear();

	if (m_enableEstPlot) plotNodesAsSpheres(toEigenMatrix(obj->getPoints()), obj->getColors(), vis, stdev, m_estPlot);
	else m_estPlot->clear();
	if (m_enableEstTransPlot) plotNodesAsSpheres(toEigenMatrix(obj->getPoints()), objFeatures->getFeatures(FeatureExtractor::FT_LAB), vis, stdev, m_estTransPlot);
	else m_estTransPlot->clear();

	if (m_enableCorrPlot) drawCorrLines(m_corrPlot, toBulletVectors(objFeatures->getFeatures(FeatureExtractor::FT_XYZ)), toBulletVectors(obsFeatures->getFeatures(FeatureExtractor::FT_XYZ)), pZgivenC, 0.01, m_nodeCorrPlot);
	else m_corrPlot->clear();

}
