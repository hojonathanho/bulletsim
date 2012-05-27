#include "clouds/utils_cv.h"
#include "clouds/utils_pcl.h"
#include "perception/apply_impulses.h"
#include "perception/config_perception.h"
#include "perception/get_nodes.h"
#include "perception/make_bodies.h"
#include "perception/optimization_forces.h"
#include "perception/visibility.h"
#include "perception/tracking_ros.h"
#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "simulation/softbodies.h"
#include "utils/vector_io.h"
#include <boost/foreach.hpp>
#include <pcl/common/transforms.h>
#include "clouds/geom.h"
#include <algorithm>

using namespace std;
using namespace Eigen;


bool allTrue(const vector<bool>& x) {
  bool out = true;
  BOOST_FOREACH(bool b, x) out &= b;
  return out;
}


void toggleKinect() {TrackingConfig::showKinect = !TrackingConfig::showKinect;}
void toggleEst() {TrackingConfig::showEst = !TrackingConfig::showEst;}
void toggleObs() {TrackingConfig::showObs = !TrackingConfig::showObs;}
void toggleLines() {TrackingConfig::showLines = !TrackingConfig::showLines;}

    
Tracker::Tracker() :
  pendingMessage(false),
  m_kinectPts(new PointCloudPlot(2)),
  m_estPlot(new PlotSpheres()),
  m_obsPlot(new PointCloudPlot(5)),
  m_corrLines(new PlotLines(3))
{

  makeScene();
  
  m_scene->env->add(m_kinectPts);
  m_scene->env->add(m_estPlot);
  m_scene->env->add(m_obsPlot);
  m_scene->env->add(m_corrLines);

  m_obsPlot->setDefaultColor(0,1,0,.5);
  m_corrLines->setDefaultColor(1,1,0,.33);

  m_scene->addVoidKeyCallback('L',&toggleLines);
  m_scene->addVoidKeyCallback('E',&toggleEst);
  m_scene->addVoidKeyCallback('O',&toggleObs);
  m_scene->addVoidKeyCallback('K',&toggleKinect);
      
  m_scene->startViewer();

}


void Tracker::makeScene() {
  m_scene = new Scene();
}

Scene* Tracker::getScene() {
  return m_scene;
}

void Tracker2::runOnline() {

  if (TrackingConfig::startIdle) {
    m_scene->step(0);
    m_scene->idle(true);
  }

  for (int t=0; ; t++) {
    int iter=0;
    beforeIterations();
    do {
      doIteration();
    } 
    while (!pendingMessage);
    afterIterations();
  }
}


void Tracker2::updateAllPlots(const std::vector<btVector3>& obsPts, const std::vector<btVector3>& estPts, const Eigen::VectorXf& sigs, const Eigen::VectorXf& pVis, const SparseArray& corr, const Eigen::VectorXf& inlierFrac) {
  if (TrackingConfig::showLines) drawCorrLines(m_corrLines, estPts, obsPts, corr);
  else m_corrLines->clear();
  if (TrackingConfig::showObs) plotObs(obsPts, inlierFrac, m_obsPlot);
  else m_obsPlot->clear();
  if (TrackingConfig::showEst) plotNodesAsSpheres(estPts, pVis, sigs, m_estPlot);
  else m_estPlot->clear();      
}
    
    
CoordinateTransformer* loadTable(Scene& scene) { // load table from standard location and add it to the scene
  MatrixXf cornersWorld = getTableCornersRansac(cloud);

  vector< vector<float> > vv = floatMatFromFile(onceFile("table_corners.txt").string());
  vector<btVector3> tableCornersCam = toBulletVectors(vv);
  CoordinateTransformer* CT = new CoordinateTransformer(getCamToWorldFromTable(tableCornersCam));

  vector<btVector3> tableCornersWorld = CT->toWorldFromCamN(tableCornersCam);
  BulletObject::Ptr table = makeTable(tableCornersWorld, .1*METERS);
  table->setColor(0,0,1,.25);
  scene.env->add(table);  
  
  return CT;
  
}    
    
