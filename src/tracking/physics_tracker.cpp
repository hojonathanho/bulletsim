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
#include "simulation/util.h"
#include <cv.h>
#include "clouds/cloud_ops.h"

using namespace Eigen;
using namespace std;

//#define CHECK_CORRECTNESS

PhysicsTracker::PhysicsTracker(TrackedObjectFeatureExtractor::Ptr object_features, FeatureExtractor::Ptr observation_features,
		VisibilityInterface::Ptr visibility_interface, ObservationVisibility::Ptr observation_visibility) :
	m_objFeatures(object_features),
	m_obsFeatures(observation_features),
	m_visInt(visibility_interface),
	m_obsVis(observation_visibility)
{
	m_priorDist = m_objFeatures->m_obj->getPriorDist();
	m_stdev = m_priorDist.transpose().replicate(m_objFeatures->m_obj->m_nNodes, 1);

	m_outlierDist = m_objFeatures->m_obj->getOutlierDist();
}

// Before calling this function, the inputs of the FeatureExtractors should be updated (if any)
void PhysicsTracker::updateFeatures() {
	m_objFeatures->updateFeatures();
	m_obsFeatures->updateFeatures();
	//shift the point cloud in the z coordinate
	//m_obsFeatures->getFeatures(FE::FT_XYZ).col(2) += VectorXf::Ones(m_obsFeatures->getFeatures(FE::FT_XYZ).rows()) * 0.01*METERS;
	for (int i=0; i<m_obsFeatures->getFeatures(FE::FT_XYZ).rows(); i++)
		if (m_obsFeatures->getFeatures(FE::FT_XYZ)(i,2) < 0.005*METERS)
			m_obsFeatures->getFeatures(FE::FT_XYZ)(i,2) = 0.005*METERS;

	m_estPts = m_objFeatures->getFeatures();
	m_obsPts = m_obsFeatures->getFeatures();

	m_vis = m_visInt->checkNodeVisibility(m_objFeatures->m_obj);

	m_vis += 0.1*VectorXf::Ones(m_vis.size());
}

void PhysicsTracker::expectationStep() {

  boost::posix_time::ptime e_time = boost::posix_time::microsec_clock::local_time();
  m_pZgivenC = calculateResponsibilities(m_estPts, m_obsPts, m_stdev, m_vis, m_objFeatures->m_obj->getOutlierDist(), m_objFeatures->m_obj->getOutlierStdev());
  LOG_TRACE("E time " << (boost::posix_time::microsec_clock::local_time() - e_time).total_milliseconds());

#if 0
	if (isFinite(m_estPts) && isFinite(m_obsPts) && isFinite(m_vis) && isFinite(m_stdev)) {
////		float a = 0.1;
////		float b = 0.9;
////		VectorXf alpha = a + m_vis.array() * (b-a);
//		VectorXf alpha = m_vis;
//		if (m_pZgivenC.rows()!=0) alpha += m_pZgivenC.rowwise().sum();
//		VectorXf expectedPi = alpha/alpha.sum();
//		m_pZgivenC = calculateResponsibilitiesNaive(m_estPts, m_obsPts, m_stdev, expectedPi, m_objFeatures->m_obj->getOutlierDist(), m_objFeatures->m_obj->getOutlierStdev());

		m_pZgivenC = calculateResponsibilitiesNaive(m_estPts, m_obsPts, m_stdev, m_vis, m_objFeatures->m_obj->getOutlierDist(), m_objFeatures->m_obj->getOutlierStdev());
	} else
		cout << "WARNING: PhysicsTracker: the input is not finite" << endl;
#endif


#ifdef CHECK_CORRECTNESS
  boost::posix_time::ptime en_time = boost::posix_time::microsec_clock::local_time();
  MatrixXf pZgivenC_naive = calculateResponsibilitiesNaive(m_estPts, m_obsPts, m_stdev, m_vis, m_objFeatures->m_obj->getOutlierDist(), m_objFeatures->m_obj->getOutlierStdev());
  cout << "E naive time " << (boost::posix_time::microsec_clock::local_time() - en_time).total_milliseconds() << endl;
  assert(isApproxEq(pZgivenC_naive, m_pZgivenC));
#endif

  if (m_obsVis)
  	expectationStepExtended();
}

// incorporates the new model of free space
void PhysicsTracker::expectationStepExtended() {
	boost::posix_time::ptime e_time = boost::posix_time::microsec_clock::local_time();

	MatrixXf nodes = FE::activeFeatures2Feature(m_estPts, FE::FT_XYZ);

//	TrackingConfig::downsample
//	TrackingConfig::pointOutlierDist
	int sample_length = 4;
	MatrixXf grid(pow(sample_length,3),3);
	int i=0;
	for (int u=0; u<sample_length; u++) {
		for (int v=0; v<sample_length; v++) {
			for (int w=0; w<sample_length; w++) {
				grid.row(i) = Vector3f(u,v,w);
				i++;
			}
		}
	}
	grid = (grid.array() - ((float)sample_length-1.0)/2.0) * TrackingConfig::downsample*METERS;

	noObsCloud.reset(new ColorCloud());
	for (int k=0; k<nodes.rows(); k++) {
		MatrixXf shifted_grid = grid + nodes.row(k).replicate(grid.rows(), 1);
		for (int i=0; i<shifted_grid.rows(); i++) {
			noObsCloud->push_back(toColorPoint(shifted_grid.row(i)));
		}
	}
	noObsCloud = downsampleCloud(noObsCloud, TrackingConfig::downsample*METERS);

	VectorXf vis = m_obsVis->checkObservationVisibility(noObsCloud);
	for (int n=0; n<noObsCloud->size(); n++) {
		if (vis(n)) {
			noObsCloud->at(n).r = 255;
			noObsCloud->at(n).g = 0;
			noObsCloud->at(n).b = 0;
		} else {
			noObsCloud->at(n).r = 0;
			noObsCloud->at(n).g = 128;
			noObsCloud->at(n).b = 128;
		}
	}
	for (int n=noObsCloud->size()-1; n>=0; n--) {
		if (!vis(n)) {
			noObsCloud->erase(noObsCloud->begin()+n);
		}
	}
	m_noObsPts.resize(noObsCloud->size(), 3);
	for (int n=0; n<noObsCloud->size(); n++) {
		m_noObsPts.row(n) = toEigenVector(noObsCloud->at(n));
	}

  m_pZgivenNoC = calculateResponsibilities(nodes, m_noObsPts, MatrixXf::Constant(nodes.rows(), 3, TrackingConfig::pointPriorDistNoObs*METERS),
  		VectorXf::Constant(nodes.rows(), 1), VectorXf::Constant(3,0), VectorXf::Constant(3, 1));


	LOG_TRACE("E2 time " << (boost::posix_time::microsec_clock::local_time() - e_time).total_milliseconds());
}

void PhysicsTracker::maximizationStep(bool apply_evidence) {

  boost::posix_time::ptime evidence_time = boost::posix_time::microsec_clock::local_time();
  if (apply_evidence) {
  	m_objFeatures->m_obj->applyEvidence(m_pZgivenC, m_obsPts);
  	if (m_obsVis) m_objFeatures->m_obj->applyEvidence(m_pZgivenNoC, m_noObsPts);
  }
  LOG_DEBUG("Evidence time " << (boost::posix_time::microsec_clock::local_time() - evidence_time).total_milliseconds());

  boost::posix_time::ptime m_time = boost::posix_time::microsec_clock::local_time();
  m_stdev = calculateStdev(m_estPts, m_obsPts, m_pZgivenC, m_priorDist, TrackingConfig::pointPriorCount);
  LOG_DEBUG("M time " << (boost::posix_time::microsec_clock::local_time() - m_time).total_milliseconds());

#if 0
  //boost::posix_time::ptime evidence_time = boost::posix_time::microsec_clock::local_time();

	if (apply_evidence && isFinite(m_pZgivenC) && isFinite(m_estPts) && isFinite(m_obsPts))
		m_objFeatures->m_obj->applyEvidence(m_pZgivenC, m_obsFeatures->getFeatures(FE::FT_XYZ));
  //cout << "Evidence time " << (boost::posix_time::microsec_clock::local_time() - evidence_time).total_milliseconds() << endl;

  //boost::posix_time::ptime m_time = boost::posix_time::microsec_clock::local_time();
  if (isFinite(m_pZgivenC)) m_stdev = calculateStdev(m_estPts, m_obsPts, m_pZgivenC, m_priorDist, 1);
  //cout << "M time " << (boost::posix_time::microsec_clock::local_time() - m_time).total_milliseconds() << endl;
#endif

#ifdef CHECK_CORRECTNESS
  boost::posix_time::ptime mn_time = boost::posix_time::microsec_clock::local_time();
  MatrixXf stdev_naive = calculateStdevNaive(m_estPts, m_obsPts, m_pZgivenC, m_priorDist, 2);
  cout << "M naive time " << (boost::posix_time::microsec_clock::local_time() - mn_time).total_milliseconds() << endl;
  assert(isApproxEq(stdev_naive, m_stdev));
#endif

  if (m_obsVis) {
		// hack so that the cloth doesn't go through the table when forces are applying from it in the direction of the table
		BulletSoftObject::Ptr bso = boost::dynamic_pointer_cast<BulletSoftObject>(m_objFeatures->m_obj->m_sim);
		if (bso) {
			btSoftBody::tNodeArray& nodes = bso->softBody->m_nodes;
			for (int i=0; i<nodes.size(); i++) {
				if ((nodes[i].m_x.z() < 0.01*METERS) && (nodes[i].m_f.z() < 0)) {
					nodes[i].m_f = btVector3(nodes[i].m_f.x(), nodes[i].m_f.y(), 0.0);
				}
			}
		}
  }

  m_objFeatures->m_obj->m_sim->getEnvironment()->step(.03,2,.015);
}


PhysicsTrackerVisualizer::PhysicsTrackerVisualizer(Scene* scene, PhysicsTracker::Ptr tracker) :
	m_scene(scene),
	m_tracker(tracker),

	m_obsInlierPlot(new PointCloudPlot(3)),
	m_obsPlot(new PointCloudPlot(6)),
	m_obsTransPlot(new PointCloudPlot(6)),
	m_noObsPlot(new PointCloudPlot(6)),
	m_estPlot(new PlotSpheres()),
	m_estTransPlot(new PlotSpheres()),
	m_estCalcPlot(new PlotSpheres()),
	m_visPlot(new PlotSpheres()),
	m_corrPlot(new PlotLines()),
	m_hiddenNodeCorrPlot(new PlotLines()),

	m_enableObsInlierPlot(false),
	m_enableObsPlot(false),
	m_enableObsTransPlot(false),
	m_enableNoObsPlot(false),
	m_enableEstPlot(false),
	m_enableEstTransPlot(false),
	m_enableEstCalcPlot(false),
	m_enableVisPlot(false),
	m_enableCorrPlot(false),
	m_enableHiddenCorrPlot(false),
	m_nodeCorrPlot(-1)
{
	m_corrPlot->setDefaultColor(1,1,0,.3);

	m_scene->addVoidKeyCallback('c',boost::bind(toggle, &m_enableCorrPlot), "correspondence lines");
	m_scene->addVoidKeyCallback('C',boost::bind(toggle, &m_enableHiddenCorrPlot), "correspondence lines");
	m_scene->addVoidKeyCallback('e',boost::bind(toggle, &m_enableEstPlot), "plot node positions");
	m_scene->addVoidKeyCallback('E',boost::bind(toggle, &m_enableEstTransPlot), "plot node positions (lab colorspace)");
	m_scene->addVoidKeyCallback('n',boost::bind(toggle, &m_enableNoObsPlot), "plot no observation cloud");
	m_scene->addVoidKeyCallback('o',boost::bind(toggle, &m_enableObsPlot), "plot observations");
	m_scene->addVoidKeyCallback('O',boost::bind(toggle, &m_enableObsTransPlot), "plot observations (lab colorspace)");
	m_scene->addVoidKeyCallback('i',boost::bind(toggle, &m_enableObsInlierPlot), "plot observations colored by inlier frac");
	m_scene->addVoidKeyCallback('v',boost::bind(toggle, &m_enableVisPlot), "plot visibility variable at the estimated nodes");
	m_scene->addVoidKeyCallback('r',boost::bind(toggle, &m_enableEstCalcPlot), "plot target positions for nodes");

  m_scene->addVoidKeyCallback('[',boost::bind(add, &m_nodeCorrPlot, -1, 0), "toggle between all and each of the correspondence lines");
  m_scene->addVoidKeyCallback(']',boost::bind(add, &m_nodeCorrPlot, 1, 0), "toggle between all and each of the correspondence lines");
}

void PhysicsTrackerVisualizer::update() {
  MatrixXf estPts, stdev, obsPts, pZgivenC;
  VectorXf vis;
  FeatureExtractor::Ptr obsFeatures;
	TrackedObjectFeatureExtractor::Ptr objFeatures;
  m_tracker->getState(estPts, stdev, obsPts, pZgivenC, vis, obsFeatures, objFeatures);

  TrackedObject::Ptr obj = objFeatures->m_obj;

	if (m_enableObsInlierPlot) {
		plotObs(toBulletVectors(obsFeatures->getFeatures(FE::FT_XYZ)), pZgivenC.colwise().sum(), m_obsInlierPlot);
		m_scene->addDrawOnce(m_obsInlierPlot);
	} else m_obsInlierPlot->clear();

	if (m_enableObsPlot) {
		plotObs(obsFeatures->getFeatures(FE::FT_XYZ), obsFeatures->getFeatures(FE::FT_BGR), m_obsPlot);
		m_scene->addDrawOnce(m_obsPlot);
	} else m_obsPlot->clear();
	if (m_enableObsTransPlot) {
		plotObs(obsFeatures->getFeatures(FE::FT_XYZ), obsFeatures->getFeatures(FE::FT_LAB), m_obsTransPlot);
		m_scene->addDrawOnce(m_obsTransPlot);
	} else m_obsTransPlot->clear();

	if (m_enableNoObsPlot) {
		m_noObsPlot->setPoints1(m_tracker->noObsCloud, 1);
		m_scene->addDrawOnce(m_noObsPlot);
	} else m_noObsPlot->clear();

	if (m_enableEstPlot) {
		plotNodesAsSpheres(toEigenMatrix(obj->getPoints()), obj->getColors(), vis, FE::activeFeatures2Feature(stdev, FE::FT_XYZ), m_estPlot);
		m_scene->addDrawOnce(m_estPlot);
	} else m_estPlot->clear();
	if (m_enableEstTransPlot) {
		plotNodesAsSpheres(toEigenMatrix(obj->getPoints()), objFeatures->getFeatures(FE::FT_LAB), vis, FE::activeFeatures2Feature(stdev, FE::FT_XYZ), m_estTransPlot);
		m_scene->addDrawOnce(m_estTransPlot);
	}	else m_estTransPlot->clear();

	if (m_enableEstCalcPlot) {
	    MatrixXf nodes = calculateNodesNaive(estPts, obsPts, pZgivenC);
	    plotNodesAsSpheres(FE::activeFeatures2Feature(nodes, FE::FT_XYZ), MatrixXf::Ones(nodes.rows(),3), VectorXf::Ones(nodes.rows())*0.5, FE::activeFeatures2Feature(stdev, FE::FT_XYZ), m_estCalcPlot);
	    //plotNodesAsSpheres(FE::activeFeatures2Feature(nodes, FE::FT_XYZ), FE::activeFeatures2Feature(nodes, FE::FT_LAB), VectorXf::Ones(nodes.rows()), FE::activeFeatures2Feature(stdev, FE::FT_XYZ), m_estCalcPlot);
			m_scene->addDrawOnce(m_estCalcPlot);
	} else m_estCalcPlot->clear();

	if (m_enableVisPlot) {
		plotNodesAsSpheres(toEigenMatrix(obj->getPoints()), vis.replicate(1,3), VectorXf::Ones(vis.size()) * 0.3, FE::activeFeatures2Feature(stdev, FE::FT_XYZ), m_visPlot);
		m_scene->addDrawOnce(m_visPlot);
	} else m_visPlot->clear();

	if (m_enableCorrPlot) {
		drawCorrLines(m_corrPlot, toBulletVectors(objFeatures->getFeatures(FE::FT_XYZ)), toBulletVectors(obsFeatures->getFeatures(FE::FT_XYZ)), pZgivenC, 0.01, m_nodeCorrPlot);
		m_scene->addDrawOnce(m_corrPlot);
	} else m_corrPlot->clear();

	if (m_enableHiddenCorrPlot && m_tracker->m_noObsPts.rows()!=0) {
		drawCorrLines(m_hiddenNodeCorrPlot, toBulletVectors(objFeatures->getFeatures(FE::FT_XYZ)), toBulletVectors(m_tracker->m_noObsPts), m_tracker->m_pZgivenNoC, 0.01, -1);
		m_scene->addDrawOnce(m_hiddenNodeCorrPlot);
	} else m_hiddenNodeCorrPlot->clear();

}
