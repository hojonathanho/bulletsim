#include "simple_physics_tracker.h"
#include "sparse_utils.h"
#include "config_tracking.h"
#include "algorithm_common.h"
#include "plotting_tracking.h"
#include "utils/conversions.h"
#include "utils/clock.h"
#include <fstream>

using namespace Eigen;
using namespace std;

SimplePhysicsTracker::SimplePhysicsTracker(TrackedObject::Ptr obj, VisibilityInterface* visInt, Environment::Ptr env) :
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
	m_enableDebugPlot(false)
{
	m_env->add(m_obsInlierPlot);
	m_env->add(m_obsPlot);
	m_env->add(m_obsTransPlot);
	m_env->add(m_estPlot);
	m_env->add(m_estTransPlot);
	m_env->add(m_corrPlot);
	m_env->add(m_debugPlot);
	m_corrPlot->setDefaultColor(1,1,0,.3);

	m_prior_dist = m_obj->getPriorDist();
	m_stdev = m_prior_dist.transpose().replicate(m_obj->m_nNodes+1, 1);

	m_outlier_dist = m_obj->getOutlierDist();
}

void SimplePhysicsTracker::updateInput(ColorCloudPtr obsPts) {
	m_obsPts = m_obj->extractFeatures(obsPts);
	float min_z = 0.015 * METERS;
  for (int i=0; i<m_obsPts.rows(); i++)
  	if (m_obsPts(i,2) < min_z)
  		m_obsPts(i,2) = min_z;
	m_obsCloud = obsPts;
}

void SimplePhysicsTracker::doIteration() {
  VectorXf vis = m_visInt->checkNodeVisibility(m_obj);
  //VectorXf vis = VectorXf::Ones(m_obj->m_nNodes);
  m_estPts = m_obj->getFeatures();
  MatrixXf pZgivenB(m_estPts.rows()+1, m_obsPts.rows());
  SparseMatrixf corr(m_estPts.rows(), m_obsPts.rows());

  StartClock();
  // E STEP
  estimateCorrespondence(m_estPts, m_stdev, vis, m_obsPts, m_outlier_dist, pZgivenB, corr);
  cout << "estimateCorrespondence " << GetClock() << endl;

  VectorXf inlierFrac = colSums(corr);
  if (m_enableObsInlierPlot) plotObs(toBulletVectors(m_obsPts.leftCols(3)), inlierFrac, m_obsInlierPlot);
	else m_obsInlierPlot->clear();
	if (m_enableObsPlot) plotObs(m_obj->featuresUntransform(m_obsPts), m_obsPlot);
	else m_obsPlot->clear();
	if (m_enableObsTransPlot) plotObs(m_obsPts, m_obsTransPlot);
	else m_obsTransPlot->clear();
	if (m_enableEstPlot) plotNodesAsSpheres(m_obj->featuresUntransform(m_estPts), vis, m_stdev.topRows(m_stdev.rows()-1), m_estPlot);
	else m_estPlot->clear();
	if (m_enableEstTransPlot) plotNodesAsSpheres(m_estPts, vis, m_stdev.topRows(m_stdev.rows()-1), m_estTransPlot);
	else m_estTransPlot->clear();
	if (m_enableCorrPlot) drawCorrLines(m_corrPlot, toBulletVectors(m_estPts.leftCols(3)), toBulletVectors(m_obsPts.leftCols(3)), corr);
	else m_corrPlot->clear();
  if (m_enableDebugPlot) plotObsBorder(m_obj->featuresUntransform(m_obsPts), m_debugPlot);
  //if (m_enableDebugPlot) plotObs(m_obsDebug, m_debugPlot);
	else m_debugPlot->clear();

	StartClock();
  // M STEP
  m_obj->applyEvidence(corr, m_obsPts);
  cout << "applyEvidence " << GetClock() << endl;
  //m_stdev = calcSigs(corr, m_estPts, m_obsPts, m_prior_dist, 1);

//  MatrixXf m_stdev2 = m_stdev;
//  m_stdev2 = calcSigsEigen(corr, m_estPts, m_obsPts, m_prior_dist, 1);
//  isApproxEq(m_stdev, m_stdev2);
//  cout << "mean " << m_stdev.colwise().mean() << "\t" << m_stdev.row(0) << endl;

  StartClock();
  m_env->step(.03,2,.015);
  cout << "step " << GetClock() << endl;
}
