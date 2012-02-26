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
#include <algorithm>
using namespace std;
using namespace Eigen;

bool allTrue(const vector<bool>& x) {
  BOOST_FOREACH(bool b, x) if (!b) return false;
  return true;
}

void toggleKinect() {TrackingConfig::showKinect = !TrackingConfig::showKinect;}
void toggleEst() {TrackingConfig::showEst = !TrackingConfig::showEst;}
void toggleObs() {TrackingConfig::showObs = !TrackingConfig::showObs;}
void toggleLines() {TrackingConfig::showLines = !TrackingConfig::showLines;}

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

  m_scene->addVoidKeyCallback('L',&toggleLines);
  m_scene->addVoidKeyCallback('E',&toggleEst);
  m_scene->addVoidKeyCallback('O',&toggleObs);
  m_scene->addVoidKeyCallback('K',&toggleKinect);

}

void Vision::runOffline() {
  m_scene->startViewer();

  if (TrackingConfig::startIdle) m_scene->idle(true);
  for (int t=0; ; t++) {
    int iter=0;
    m_gotEm = vector<bool>(m_msgs.size(), false);
    ENSURE(recvAll());

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
    m_sigs.setConstant(sq(.025*METERS));
  }


  MatrixXf estPtsEigen = toEigenMatrix(m_towelEstPts);
  MatrixXf obsPtsEigen = toEigenMatrix(m_towelObsPts);

  assert(isFinite(obsPtsEigen));
  assert(isFinite(estPtsEigen));

  Eigen::MatrixXf corrEigen = calcCorrProb(estPtsEigen, m_sigs, obsPtsEigen, pVis, TrackingConfig::outlierParam);
  SparseArray corr = toSparseArray(corrEigen, TrackingConfig::cutoff);
  m_sigs = calcSigs(corr, estPtsEigen, obsPtsEigen, .025*METERS, 1);

  vector<btVector3> impulses = calcImpulsesDamped(m_towelEstPts, towelEstVels, m_towelObsPts, corr,  getNodeMasses(m_towel), TrackingConfig::kp, TrackingConfig::kd);
  vector<float> ms = getNodeMasses(m_towel);
  for (int node=0; node<impulses.size(); node++) m_towel->softBody->addForce(impulses[node],node);

  if (TrackingConfig::showLines) drawCorrLines(m_corrLines, m_towelEstPts, m_towelObsPts, corr);
  if (TrackingConfig::showObs) plotObs(corrEigen, m_towelObsPts, m_obsPlot);
  if (TrackingConfig::showEst) plotNodesAsSpheres(m_towel->softBody.get(), pVis, m_sigs, m_estPlot);


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
  m_ropeMask = labels < 2;
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
    m_sigs.setConstant(sq(.025*METERS));
  }

  MatrixXf estPtsEigen = toEigenMatrix(m_ropeEstPts);
  MatrixXf obsPtsEigen = toEigenMatrix(m_ropeObsPts);

  Eigen::MatrixXf corrEigen = calcCorrProb(estPtsEigen, m_sigs, obsPtsEigen, pVis, TrackingConfig::outlierParam); // todo: use new version with var est

  SparseArray corr = toSparseArray(corrEigen, TrackingConfig::cutoff);
  vector<float> masses = getNodeMasses(m_rope);
  vector<btVector3> ropeVel = getNodeVels(m_rope);
  vector<btVector3> impulses = calcImpulsesDamped(m_ropeEstPts, ropeVel, m_ropeObsPts, corr, masses, TrackingConfig::kp, TrackingConfig::kd); // todo: use new version with damping
  m_sigs = calcSigs(corr, estPtsEigen, obsPtsEigen, .025*METERS, 1);
  applyImpulses(impulses, m_rope);


  if (TrackingConfig::showLines) drawCorrLines(m_corrLines, m_ropeEstPts, m_ropeObsPts, corr);
  if (TrackingConfig::showObs) plotObs(corrEigen, m_ropeObsPts, m_obsPlot);
  //if (TrackingConfig::showEst) plotNodesAsSpheres(m_rope->softBody.get(), pVis, m_sigs, m_estPlot);


  m_scene->env->step(.03,2,.015);
  m_scene->draw();

}

void RopeVision::afterIterations() {
  vector< vector<float> > vv = toVecVec(m_ropeEstPts);
  m_pub->send(VecVecMessage<float>(vv));
}

template <typename MatrixT>
void readMatrix(istream& is, MatrixT& m) {
  int nRows, nCols;
  is >> nRows >> nCols;
  cout << "nRows,nCols" << nRows << " " << nCols << endl;
  m.resize(nRows, nCols);
  for (int row=0; row < nRows; row++)
    for (int col=0; col < nCols; col++) {
      float x;
      is >> x;
      m(row,col) = x;
    }
}

void RopeInitMessage::readDataFrom(path p) {
  ifstream infile(p.string().c_str());
  if (infile.fail()) throw FileOpenError(p.string());
  readMatrix(infile, m_data.m_positions);
  readMatrix(infile, m_data.m_labels);
  infile.close();
}

void RopeInitMessage::writeDataTo(path p) {
  throw runtime_error("not implemented");
}

History::History(int maxsize) : m_maxsize(maxsize), m_sum(0) {}
void History::put(float x) { 
  if (size() == m_maxsize) {
    m_sum -= m_data.front();
    m_data.pop();
  }
  m_sum += x;
  m_data.push(x);
}
float History::sum() {return m_sum;}
int History::size() {return m_data.size();}
bool History::full() {return m_data.size() == m_maxsize;}


MatrixXf TrackedRope::featsFromCloud(ColorCloudPtr cloud) {
  MatrixXf out(cloud->size(), 4);
  for (int i=0; i < cloud->points.size(); i++) {
    pcl::PointXYZRGB& pt = cloud->points[i];
    out(i,0) = pt.x;
    out(i,1) = pt.y;
    out(i,2) = pt.z;
    out(i,3) = 1;
  }
  return out;
}

MatrixXf TrackedRope::featsFromSim() {
  MatrixXf out(m_sim->children.size(),4);
  for (int i=0; i < m_sim->children.size(); i++) {
    btVector3 pos = m_sim->children[i]->rigidBody->getCenterOfMassPosition();
    out(i,0) = pos.x();
    out(i,1) = pos.y();
    out(i,2) = pos.z();
    out(i,3) = m_labels[i];
  }
  return out;
}

void TrackedRope::applyEvidence(const SparseArray& corr, const MatrixXf& obsPts) {
  vector<float> masses = getNodeMasses(m_sim);
  vector<btVector3> ropePos = m_sim->getNodes();
  vector<btVector3> ropeVel = getNodeVels(m_sim);
  vector<btVector3> impulses = calcImpulsesDamped(ropePos, ropeVel, toBulletVectors(obsPts), corr, masses, TrackingConfig::kp, TrackingConfig::kd);
  m_sigs = calcSigs(corr, toEigenMatrix(ropePos), obsPts, .025*METERS, 1);

  VectorXf forceNorms = toEigenMatrix(impulses).rowwise().norm();
  m_forceHistory.put(forceNorms.lpNorm<1>());
  applyImpulses(impulses, m_sim);  
}

TrackedRope::TrackedRope(const RopeInitMessage& message, CoordinateTransformer* CT) {
  vector<btVector3> positions = CT->toWorldFromCamN(toBulletVectors(message.m_data.m_positions));
  VectorXf labels = message.m_data.m_labels;
  init(positions, labels);
}

void TrackedRope::init(const vector<btVector3>& nodes, const VectorXf& labels) {
  m_labels = labels;
  m_sim.reset(new CapsuleRope(nodes,.0075*METERS));
  m_sigs.resize(m_labels.rows(),1);
  m_sigs.setConstant(sq(.025*METERS));
}

RopeVision2::RopeVision2() {
  setupComm();
  setupScene();
}

osg::Vec4f hyp_colors[6] = {osg::Vec4f(1,0,0,.3),
			    osg::Vec4f(0,1,0,.3),
			    osg::Vec4f(0,0,1,.3),
			    osg::Vec4f(1,1,0,.3),
			    osg::Vec4f(0,1,1,.3),
			    osg::Vec4f(1,0,1,.3)
};


void RopeVision2::setupComm() {
  // not each frame
  m_ropeInitSub = new FileSubscriber("rope_init","txt"); // not adding stuff because it's not called every step
  // ------------

  // each frame -------------
  m_ropeSub = new FileSubscriber("rope_pts","pcd");
  m_subs.push_back(m_ropeSub);
  m_msgs.push_back(&m_ropePtsMsg);
  
  m_labelSub = new FileSubscriber("labels","png");
  m_subs.push_back(m_labelSub);
  m_msgs.push_back(&m_labelMsg);
  // ----------------
  
  m_pub = new FilePublisher("rope_model", "txt");

  m_obsCloud.reset(new ColorCloud());

}

void RopeVision2::setupScene() {
  vector< vector<float> > vv = floatMatFromFile(onceFile("table_corners.txt").string());
  vector<btVector3> tableCornersCam = toBulletVectors(vv);
  m_CT = new CoordinateTransformer(getCamToWorldFromTable(tableCornersCam));

  vector<btVector3> tableCornersWorld = m_CT->toWorldFromCamN(tableCornersCam);
  m_table = makeTable(tableCornersWorld, .1*METERS);
  m_table->setColor(0,0,1,.25);

  m_scene->env->add(m_table);
}

void RopeVision2::beforeIterations() {
  ColorCloudPtr cloudCam  = m_kinectMsg.m_data;
  ColorCloudPtr cloudWorld(new ColorCloud());
  pcl::transformPointCloud(*cloudCam, *cloudWorld, m_CT->worldFromCamEigen);
  if (TrackingConfig::showKinect) m_kinectPts->setPoints1(cloudWorld);
  else {m_kinectPts->clear(); cout << "hihihih" << endl;}
  cv::Mat labels = toSingleChannel(m_labelMsg.m_data);
  m_ropeMask = labels < 2;
  m_depthImage = getDepthImage(cloudCam);
  pcl::transformPointCloud(*m_ropePtsMsg.m_data, *m_obsCloud, m_CT->worldFromCamEigen);
  m_obsFeats = TrackedRope::featsFromCloud(m_obsCloud);


  bool gotRope = m_ropeInitSub->recv(m_ropeInitMsg,false);
  if (gotRope) {
    TrackedRope::Ptr trackedRope(new TrackedRope(m_ropeInitMsg, m_CT));
    cout << "currently " << m_ropeHypoths.size() << " hypotheses" << endl;
    osg::Vec4f color = hyp_colors[m_ropeHypoths.size()];
    BOOST_FOREACH(BulletObject::Ptr bo, trackedRope->m_sim->children) bo->setColor(color[0], color[1], color[2], color[3]);
    addRopeHypoth(trackedRope);
  }
}

void RopeVision2::doIteration() {
  BOOST_FOREACH(TrackedRope::Ptr hyp, m_ropeHypoths) {
    VectorXf pVis = VectorXf::Ones(hyp->m_sim->nLinks,1); // TODO
    Eigen::MatrixXf corrEigen = calcCorrProb(hyp->featsFromSim(), hyp->m_sigs, m_obsFeats, pVis, TrackingConfig::outlierParam);
    SparseArray corr = toSparseArray(corrEigen, TrackingConfig::cutoff);
    hyp->applyEvidence(corr, toEigenMatrix(m_obsCloud));

    if (hyp==m_ropeHypoths[0]) {
      vector<btVector3> obsPts = toBulletVectors(m_obsCloud);
      vector<btVector3> estPts = getNodes(hyp->m_sim);
      if (TrackingConfig::showLines) drawCorrLines(m_corrLines, estPts, obsPts, corr);
      else m_corrLines->clear();
      if (TrackingConfig::showObs) plotObs(corrEigen, obsPts, m_obsPlot);
      else m_obsPlot->clear();
      if (TrackingConfig::showEst) plotNodesAsSpheres(estPts, pVis, hyp->m_sigs, m_estPlot);
      else m_estPlot->clear();
    }

  }

  BOOST_FOREACH(Fork::Ptr f, m_scene->forks)
    f->env->step(.03,2,.015);
  m_scene->draw();



}

void RopeVision2::afterIterations() {
  vector< vector<float> > vv; 
  m_pub->send(VecVecMessage<float>(vv));
  //cullHypoths();
}

void RopeVision2::cullHypoths() {  
  
  if (m_ropeHypoths.size() == 0) return;

  vector<TrackedRope::Ptr> comparableHypoths;
  BOOST_FOREACH(TrackedRope::Ptr hyp, m_ropeHypoths) {
    if (hyp->m_forceHistory.full()) comparableHypoths.push_back(hyp);
  }
  
  vector<float> costs(comparableHypoths.size());
  for (int i=0; i < m_ropeHypoths.size(); ++i) costs[i] = m_ropeHypoths[i]->m_forceHistory.sum();
  float minCost = *min_element(costs.begin(), costs.end());
  
  vector<TrackedRope::Ptr> forDeletion;
  for (int i=0; i < m_ropeHypoths.size(); ++i) if (costs[i] > 2*minCost) {
    cout << "deleting hypothesis " << i << endl;
    forDeletion.push_back(comparableHypoths[i]);
  }
  
  BOOST_FOREACH(TrackedRope::Ptr hyp, forDeletion) removeRopeHypoth(hyp);
}


void RopeVision2::addRopeHypoth(TrackedRope::Ptr ropeHypoth) {

  BulletInstance::Ptr bullet2(new BulletInstance);
  bullet2->setGravity(BulletConfig::gravity);
  OSGInstance::Ptr osg2(new OSGInstance);
  m_scene->osg->root->addChild(osg2->root.get());
  Fork::Ptr fork(new Fork(m_scene->env, bullet2, osg2));
  m_scene->registerFork(fork);
  cout << (bool)ropeHypoth->m_sim << endl;
  fork->env->add(ropeHypoth->m_sim);
  m_ropeHypoths.push_back(ropeHypoth);

}

void RopeVision2::removeRopeHypoth(TrackedRope::Ptr ropeHypoth) {
  for (vector<TrackedRope::Ptr>::iterator it=m_ropeHypoths.begin(); it < m_ropeHypoths.end(); ++it) {
    if (*it == ropeHypoth) {
      m_ropeHypoths.erase(it);
      m_scene->unregisterFork(m_rope2fork[ropeHypoth]);
      m_rope2fork.erase(ropeHypoth);
      return;
    }
  }
  throw std::runtime_error("rope hypothesis not found");
}
