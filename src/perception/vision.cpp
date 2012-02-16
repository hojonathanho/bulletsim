#include "clouds/comm_pcl.h"
#include "clouds/comm_cv.h"
#include "clouds/utils_cv.h"
#include "clouds/utils_pcl.h"
#include "comm/comm.h"
#include "perception/apply_impulses.h"
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
using namespace Eigen;

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
  m_estPlot.reset(new PlotSpheres());
  m_obsPlot.reset(new PointCloudPlot(5));
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

float sq(float x) {return x*x;}

void TowelVision::doIteration() {
  VectorXf pVis = calcVisibility(m_towel->softBody.get(), m_scene->env->bullet->dynamicsWorld, m_CT->worldFromCamUnscaled.getOrigin()*METERS);

  vector<btVector3> m_towelEstPts = getNodes(m_towel);
  vector<btVector3> towelEstVels = getNodeVels(m_towel);

  if (m_sigs.rows() == 0) {
    m_sigs.resize(m_towelEstPts.size(), 1);
    m_sigs.setConstant(.025*METERS);
  }
  if (TrackingConfig::showEst) plotNodesAsSpheres(m_towel->softBody.get(), pVis, m_sigs, m_estPlot);


  MatrixXf estPtsEigen = toEigenMatrix(m_towelEstPts);
  MatrixXf obsPtsEigen = toEigenMatrix(m_towelObsPts);

  assert(isFinite(obsPtsEigen));
  assert(isFinite(estPtsEigen));

  Eigen::MatrixXf corrEigen = calcCorrProb(estPtsEigen, m_sigs, obsPtsEigen, pVis, TrackingConfig::outlierParam);
  SparseArray corr = toSparseArray(corrEigen, TrackingConfig::cutoff);
  if (TrackingConfig::showObs) plotObs(corrEigen, m_towelObsPts, m_obsPlot);
  m_sigs = calcSigs(corr, estPtsEigen, obsPtsEigen, .025*METERS, 1);
  if (TrackingConfig::showLines) drawCorrLines(m_corrLines, m_towelEstPts, m_towelObsPts, corr);

  vector<btVector3> impulses = calcImpulsesDamped(m_towelEstPts, towelEstVels, m_towelObsPts, corr,  getNodeMasses(m_towel), 150, 30);
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

RopeVision::RopeVision() {
  setupComm();
  setupScene();
}

void RopeVision::setupComm() {
  m_ropeSub = new FileSubscriber("rope_pts","pcd");
  m_subs.push_back(m_ropeSub);
  m_msgs.push_back(&m_ropePtsMsg);
  m_labelSub = new FileSubscriber("labels","png");
  m_subs.push_back(m_labelSub);
  m_msgs.push_back(&m_labelMsg);
  m_pub = new FilePublisher("rope_model", "txt");
}

void RopeVision::setupScene() {
  vector< vector<float> > vv = floatMatFromFile(onceFile("table_corners.txt").string());
  vector<btVector3> tableCornersCam = toBulletVectors(vv);
  m_CT = new CoordinateTransformer(getCamToWorldFromTable(tableCornersCam));

  vector<btVector3> tableCornersWorld = m_CT->toWorldFromCamN(tableCornersCam);
  m_table = makeTable(tableCornersWorld, .1*METERS);
  m_table->setColor(0,0,1,.25);

  vector<btVector3> ropePtsCam = toBulletVectors(floatMatFromFile(onceFile("init_rope.txt").string()));
  m_rope.reset(new CapsuleRope(m_CT->toWorldFromCamN(ropePtsCam), .0075*METERS));

  m_scene->env->add(m_rope);
  m_scene->env->add(m_table);
}

void RopeVision::beforeIterations() {
  ColorCloudPtr cloudCam  = m_kinectMsg.m_data;
  ColorCloudPtr cloudWorld(new ColorCloud());
  pcl::transformPointCloud(*cloudCam, *cloudWorld, m_CT->worldFromCamEigen);
  if (TrackingConfig::showKinect) m_kinectPts->setPoints1(cloudWorld);

  cv::Mat labels = toSingleChannel(m_labelMsg.m_data);
  m_ropeMask = labels == 1;
  m_depthImage = getDepthImage(cloudCam);
  m_ropeObsPts = m_CT->toWorldFromCamN(toBulletVectors(m_ropePtsMsg.m_data));


}

void RopeVision::doIteration() {
  m_ropeEstPts = m_rope->getNodes();
  Eigen::MatrixXf ropePtsCam = toEigenMatrix(m_CT->toCamFromWorldN(m_ropeEstPts));
  VectorXf pVis = calcVisibility(ropePtsCam, m_depthImage, m_ropeMask);
  colorByVisibility(m_rope, pVis);


  if (m_sigs.rows() == 0) {
    m_sigs.resize(m_ropeEstPts.size(), 1);
    m_sigs.setConstant(.025*METERS);
  }

  MatrixXf estPtsEigen = toEigenMatrix(m_ropeEstPts);
  MatrixXf obsPtsEigen = toEigenMatrix(m_ropeObsPts);

  Eigen::MatrixXf corrEigen = calcCorrProb(estPtsEigen, obsPtsEigen, pVis, TrackingConfig::sigB, TrackingConfig::outlierParam); // todo: use new version with var est
  SparseArray corr = toSparseArray(corrEigen, TrackingConfig::cutoff);
  vector<btVector3> impulses = calcImpulsesSimple(m_ropeEstPts, m_ropeObsPts, corr, TrackingConfig::impulseSize); // todo: use new version with damping
  applyImpulses(impulses, m_rope);
  m_scene->env->step(.03,2,.015);
  m_scene->draw();

}

void RopeVision::afterIterations() {
  vector< vector<float> > vv = toVecVec(m_ropeEstPts);
  m_pub->send(VecVecMessage<float>(vv));
}
