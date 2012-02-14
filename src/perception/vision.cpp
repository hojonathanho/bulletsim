#include "clouds/comm_pcl.h"
#include "comm/comm.h"
#include "perception/config_perception.h"
#include "perception/get_nodes.h"
#include "perception/make_bodies.h"
#include "perception/optimization_forces.h"
#include "perception/visibility.h"
#include "perception/vision.h"
#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "simulation/softbodies.h"
#include "utils/vector_io.h"
#include <boost/foreach.hpp>
#include <pcl/common/transforms.h>
#include "clouds/geom.h"

bool allTrue(const vector<bool>& x) {
  BOOST_FOREACH(bool b, x) if (!b) return false;
  return true;
}


Vision::Vision() {
  cout << "Vision::Vision" << endl;
  setupScene();
  setupComm();
}

Vision::~Vision() {
  //todo
}

bool Vision::recvAll() {
  for (int i=0; i < m_subs.size(); i++) {
    if (!m_gotEm[i]) {
      m_gotEm[i] = m_subs[i]->recv(*m_msgs[i], false);
    }
  }
  BOOST_FOREACH(bool b, m_gotEm) cout << b << " ";
  cout << endl;
  return allTrue(m_gotEm);
}

void Vision::setupComm() {
  cout << "Vision::setupComm" << endl;
  initComm();
  m_subs.push_back(new FileSubscriber("kinect","pcd"));
  m_msgs.push_back(&m_kinectMsg);
}

void Vision::setupScene() {

  m_scene = new Scene();

  m_kinectPts.reset(new PointCloudPlot(2));
  m_estPlot.reset(new PointCloudPlot(4));
  m_obsPlot.reset(new PointCloudPlot(4));
  m_corrLines.reset(new PlotLines(3));

  m_scene->env->add(m_kinectPts);
  m_scene->env->add(m_estPlot);
  m_scene->env->add(m_obsPlot);
  m_scene->env->add(m_corrLines);

  m_obsPlot->setDefaultColor(0,1,0,.5);
  m_corrLines->setDefaultColor(1,1,0,.33);
}

void Vision::runOffline() {
  m_scene->startViewer();
  if (TrackingConfig::startIdle) m_scene->idle(true);
  for (int t=0; ; t++) {
    m_gotEm = vector<bool>(m_msgs.size(), false);
    ENSURE(recvAll());
    int iter=0;
    m_gotEm = vector<bool>(m_msgs.size());

    beforeIterations();
    do {
      cout << "iteration " << iter << endl;
      doIteration();
      iter++;
    }
    while (iter < TrackingConfig::nIter);
    afterIterations();
  }
}

void Vision::runOnline() {
  m_scene->startViewer();
  m_gotEm = vector<bool>(m_msgs.size(), false);
  ENSURE(recvAll());
  for (int t=0; ; t++) {
    int iter=0;
    m_gotEm = vector<bool>(m_msgs.size(), false);

    beforeIterations();
    do {
      cout << "iteration " << iter << endl;
      doIteration();
      iter++;
    }
    while (!recvAll());
    afterIterations();
  }
}



TowelVision::TowelVision() {
  // why do I need this? These methods are virtual.
  setupComm();
  setupScene(); 
}

void TowelVision::doIteration() {
  vector<float> pVis = calcVisibility(m_towel->softBody.get(), m_scene->env->bullet->dynamicsWorld, m_CT->worldFromCamUnscaled.getOrigin()*METERS);
  colorByVisibility(m_towel->softBody.get(), pVis, m_estPlot);

  vector<btVector3> m_towelEstPts = getNodes(m_towel);
  vector<btVector3> towelEstVels = getNodeVels(m_towel);
  SparseArray corr = toSparseArray(calcCorrProb(toEigenMatrix(m_towelEstPts), toEigenMatrix(m_towelObsPts), toVectorXf(pVis), TrackingConfig::sigB, TrackingConfig::outlierParam), TrackingConfig::cutoff);
  if (TrackingConfig::showLines) drawCorrLines(m_corrLines, m_towelEstPts, m_towelObsPts, corr);

  vector<btVector3> impulses = calcImpulsesDamped(m_towelEstPts, towelEstVels, m_towelObsPts, corr,  getNodeMasses(m_towel), 300, 60);
  vector<float> ms = getNodeMasses(m_towel);
  for (int node=0; node<impulses.size(); node++) m_towel->softBody->addForce(impulses[node],node);
  m_scene->env->step(.03,2,.015);
  m_scene->draw();
}

void TowelVision::beforeIterations() {
    if (TrackingConfig::showKinect) {
      ColorCloudPtr cloudCam  = m_kinectMsg.m_data;
      ColorCloudPtr cloudWorld(new ColorCloud());
      pcl::transformPointCloud(*cloudCam, *cloudWorld, m_CT->worldFromCamEigen);
      m_kinectPts->setPoints1(cloudWorld);
    }

    m_towelObsPts = m_CT->toWorldFromCamN(toBulletVectors(m_towelPtsMsg.m_data));
    if (TrackingConfig::showObs) m_obsPlot->setPoints(m_towelObsPts);
}

void TowelVision::afterIterations() {
  vector< vector<float> > vv = toVecVec(m_towelEstPts);
  m_pub->send(VecVecMessage<float>(vv));
}

void TowelVision::setupComm() {

  m_towelSub = new FileSubscriber("towel_pts","pcd");
  m_subs.push_back(m_towelSub);
  m_msgs.push_back(&m_towelPtsMsg);
  m_pub = new FilePublisher("towel_model", "txt");
  m_gotEm = vector<bool>(m_subs.size(), false);
}

void TowelVision::setupScene() {

  vector< vector<float> > vv = floatMatFromFile(onceFile("table_corners.txt").string());
  vector<btVector3> tableCornersCam = toBulletVectors(vv);
  m_CT = new CoordinateTransformer(getCamToWorldFromTable(tableCornersCam));

  vector<btVector3> tableCornersWorld = m_CT->toWorldFromCamN(tableCornersCam);
  m_table = makeTable(tableCornersWorld, .1*METERS);
  m_table->setColor(0,0,1,.25);

  m_towelPtsMsg.fromFiles(m_towelSub->m_names.getCur());
  vector<btVector3> towelPtsCam = toBulletVectors(m_towelPtsMsg.m_data);
  vector<btVector3> towelPtsWorld = m_CT->toWorldFromCamN(towelPtsCam);
  vector<btVector3> towelCorners = toBulletVectors(getCorners(toEigenVectors(towelPtsWorld)));
  BOOST_FOREACH(btVector3& pt, towelCorners) pt += btVector3(.01*METERS,0,0);
  m_towel = makeSelfCollidingTowel(towelCorners, m_scene->env->bullet->softBodyWorldInfo);

  m_scene->env->add(m_towel);
  m_scene->env->add(m_table);
}
