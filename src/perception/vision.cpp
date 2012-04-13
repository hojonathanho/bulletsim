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
#include <BulletSoftBody/btSoftBodyHelpers.h>

using namespace std;
using namespace Eigen;

static const float LABEL_MULTIPLIER = 100;
static const int MIN_CULL_AGE = 5;

bool allTrue(const vector<bool>& x) {
  bool out = true;
  BOOST_FOREACH(bool b, x) out &= b;
  return out;
}


void toggleKinect() {TrackingConfig::showKinect = !TrackingConfig::showKinect;}
void toggleEst() {TrackingConfig::showEst = !TrackingConfig::showEst;}
void toggleObs() {TrackingConfig::showObs = !TrackingConfig::showObs;}
void toggleLines() {TrackingConfig::showLines = !TrackingConfig::showLines;}

Vision::Vision() {
  setupScene();
  setupComm();
}

Vision::~Vision() {
  //todo
}

bool Vision::recvAll() {
  for (int i=0; i < m_subs.size(); i++)
    if (!m_gotEm[i]) m_gotEm[i] = m_subs[i]->recv(*m_msgs[i], false);
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
  m_obsPlot.reset(new PointCloudPlot(10));
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
    cout << "iteration";
    do {
      cout << " " << iter;
      doIteration();
      iter++;
    }
    while (iter < TrackingConfig::nIter);
    cout << endl;
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


void Vision::updateAllPlots(const vector<btVector3>& obsPts, const vector<btVector3>& estPts, const VectorXf& sigs, const VectorXf& pVis, const SparseArray& corr, const VectorXf& inlierFrac) {
  if (TrackingConfig::showLines) drawCorrLines(m_corrLines, estPts, obsPts, corr);
  else m_corrLines->clear();
  if (TrackingConfig::showObs) plotObs(obsPts, inlierFrac, m_obsPlot);
  else m_obsPlot->clear();
  if (TrackingConfig::showEst) plotNodesAsSpheres(estPts, pVis, sigs, m_estPlot);
  else m_estPlot->clear();      
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
  // if (TrackingConfig::showObs) plotObs(corrEigen, m_towelObsPts, m_obsPlot);
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
  m_labelSub = new FileSubscriber("labels","bmp");
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
  // if (TrackingConfig::showObs) plotObs(corrEigen, m_ropeObsPts, m_obsPlot);
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

MatrixXf TrackedRope::featsFromCloud(ColorCloudPtr cloud) {
  MatrixXf out(cloud->size(), 4);
  for (int i=0; i < cloud->points.size(); i++) {
    pcl::PointXYZRGBA& pt = cloud->points[i];
    out(i,0) = pt.x;
    out(i,1) = pt.y;
    out(i,2) = pt.z;
    out(i,3) = pt._unused*LABEL_MULTIPLIER;
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
    out(i,3) = m_labels[i]*LABEL_MULTIPLIER;
  }
  return out;
}

void TrackedRope::applyEvidence(const SparseArray& corr, const MatrixXf& obsPts) {
  vector<float> masses = getNodeMasses(m_sim);
  vector<btVector3> ropePos = getNodes(m_sim);
  vector<btVector3> ropeVel = getNodeVels(m_sim);
  vector<btVector3> impulses = calcImpulsesDamped(ropePos, ropeVel, toBulletVectors(obsPts), corr, masses, TrackingConfig::kp, TrackingConfig::kd);
  m_sigs = calcSigs(corr, toEigenMatrix(ropePos), obsPts, .1*METERS, 1);
  applyImpulses(impulses, m_sim);  
}

TrackedRope::TrackedRope(const RopeInitMessage& message, CoordinateTransformer* CT) {
  vector<btVector3> positions = CT->toWorldFromCamN(toBulletVectors(message.m_data.m_positions));
  VectorXf labels = message.m_data.m_labels;
  init(positions, labels);
}

void TrackedRope::init(const vector<btVector3>& nodes, const VectorXf& labels) {
  cout << labels.transpose() << endl;
  m_labels = labels;
  m_sim.reset(new CapsuleRope(nodes,.0075*METERS));
  m_sigs.resize(m_labels.rows(),1);
  m_sigs.setConstant(sq(.025*METERS));
}

    
Vision2::Vision2() {

  cout << "Vision2::Vision2()" << endl;
  
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
      
  m_scene->startViewer();
  OSGCamParams cp(m_CT->worldFromCamUnscaled, GeneralConfig::scale);
  m_scene->manip->setHomePosition(cp.eye, cp.center, cp.up);


}

Vision2::~Vision2() {}

void Vision2::runOnline() {
  m_multisub->prepare();
  ENSURE(m_multisub->recvAll()); // get first messages
  for (int t=0; ; t++) {
    int iter=0;
    beforeIterations();
    m_multisub->prepare();
    do {
      cout << iter++ << endl;
      doIteration();
    } 
    while (!m_multisub->recvAll());
    afterIterations();
  }
}

void Vision2::runOffline() {  
  for (int t=0; ; t++) {
    cout << "t=" << t << endl;
    int iter=0;
    m_multisub->prepare();
    ENSURE(m_multisub->recvAll());

    beforeIterations();
    do {
      doIteration();
      cout << iter++ << endl;
    }
    while (iter < TrackingConfig::nIter);
    afterIterations();
  }
}

void Vision2::updateAllPlots(const std::vector<btVector3>& obsPts, const std::vector<btVector3>& estPts, const Eigen::VectorXf& sigs, const Eigen::VectorXf& pVis, const SparseArray& corr, const Eigen::VectorXf& inlierFrac) {

  if (TrackingConfig::showLines) drawCorrLines(m_corrLines, estPts, obsPts, corr);
  else m_corrLines->clear();
  if (TrackingConfig::showObs) plotObs(obsPts, inlierFrac, m_obsPlot);
  else m_obsPlot->clear();
  if (TrackingConfig::showEst) plotNodesAsSpheres(estPts, pVis, sigs, m_estPlot);
  else m_estPlot->clear();      
}
    
    
CoordinateTransformer* loadTable(Scene& scene) { // load table from standard location and add it to the scene
  vector< vector<float> > vv = floatMatFromFile(onceFile("table_corners.txt").string());
  vector<btVector3> tableCornersCam = toBulletVectors(vv);
  CoordinateTransformer* CT = new CoordinateTransformer(getCamToWorldFromTable(tableCornersCam));

  vector<btVector3> tableCornersWorld = CT->toWorldFromCamN(tableCornersCam);
  BulletObject::Ptr table = makeTable(tableCornersWorld, .1*METERS);
  table->setColor(0,0,1,.25);
  scene.env->add(table);  
  
  return CT;
  
}    
    
inline int idx(int row,int col,int xres) {return col+row*xres;}
typedef pair<int, int> intpair;

TrackedTowel::TrackedTowel(BulletSoftObject::Ptr sim, int xres, int yres) {

  set<intpair> knots;
  node2knots = vector< vector<int> >(xres*yres);

  int decimation = 2;
  for (int row = 0; row < yres; row += decimation) {
    for (int col = 0; col < xres; col += decimation) {
      knots.insert(intpair(row,col));
      m_nodeInds.push_back(idx(row,col,xres));
    }
  }

  for (int row = 0; row < yres; row++) 
    for (int col = 0; col < xres; col++) 
      for (int dy = -1; dy <= 1; dy++)
	for (int dx = -1; dx <= 1; dx++)
	  if (knots.find(intpair(row+dy, col+dx)) != knots.end()) 
	    node2knots[idx(row,col,xres)].push_back(idx(row+dy, col+dx, xres));

  for (int i=0; i < xres*yres; i++) {
    cout << i << ": ";
    BOOST_FOREACH(int j, node2knots[i]) {
      cout << j << " ";
      assert(i==j || sim->softBody->checkLink(i,j));
    }
    cout << endl;
  }

  m_sim = sim;
  m_masses = getNodeMasses();

  m_sigs.resize(m_masses.size(),1);
  m_sigs.setConstant(sq(.025*METERS));


}

MatrixXf TrackedTowel::featsFromCloud(ColorCloudPtr cloud) {
  MatrixXf out(cloud->size(), 3);
  for (int i=0; i < cloud->points.size(); i++) {
    pcl::PointXYZRGBA& pt = cloud->points[i];
    out(i,0) = pt.x;
    out(i,1) = pt.y;
    out(i,2) = pt.z;
  }
  return out;
}

MatrixXf TrackedTowel::featsFromSim() {
  return toEigenMatrix(getNodes());
}

vector<btVector3> TrackedTowel::getNodes() {
  const btAlignedObjectArray<btSoftBody::Node>& nodes = m_sim->softBody->m_nodes;

  vector<btVector3> pos;
  pos.reserve(m_nodeInds.size());
  
  BOOST_FOREACH(int ind, m_nodeInds) pos.push_back(nodes[ind].m_x);

  return pos;
}

vector<btVector3> TrackedTowel::getNodes(vector<btVector3>& vel) {
  const btAlignedObjectArray<btSoftBody::Node>& nodes = m_sim->softBody->m_nodes;

  vector<btVector3> pos;
  vel.clear();
  pos.reserve(m_nodeInds.size());
  vel.reserve(m_nodeInds.size());

  BOOST_FOREACH(int ind, m_nodeInds) {
    const btSoftBody::Node& node = nodes[ind];
    pos.push_back(node.m_x);
    vel.push_back(node.m_v);
  }
  return pos;
}

vector<float> TrackedTowel::getNodeMasses() {

  vector<float> masses;
  masses.reserve(m_nodeInds.size());
  
  const btAlignedObjectArray<btSoftBody::Node>& nodes = m_sim->softBody->m_nodes;
  BOOST_FOREACH(int ind, m_nodeInds)
    masses.push_back(1/nodes[ind].m_im);
  
  return masses;
}


vector<int> TrackedTowel::getNodeInds() {
  return m_nodeInds;
}

void TrackedTowel::applyEvidence(const SparseArray& corr, const vector<btVector3>& obsPts) {
  vector<btVector3> estPos, estVel;
  estPos = getNodes(estVel);

  vector<btVector3> impulses = calcImpulsesDamped(estPos, estVel, obsPts, corr, m_masses, TrackingConfig::kp, TrackingConfig::kd);

  m_sigs = calcSigs(corr, toEigenMatrix(estPos), toEigenMatrix(obsPts), .1*METERS, .5);

  int nNodes = node2knots.size();
  vector<btVector3> allImpulses(nNodes, btVector3(0,0,0));
  for (int i=0; i < m_nodeInds.size(); i++) allImpulses[m_nodeInds[i]] = impulses[i];
  for (int i=0; i < nNodes; i++) {
    BOOST_FOREACH(int j, node2knots[i]) allImpulses[i] += allImpulses[j];
    allImpulses[i] /= node2knots[i].size();
  }
  applyImpulses(allImpulses, m_sim);
}



TowelVision2::TowelVision2() : m_pub("towel_model", "txt") {

  Vision2::m_multisub = m_multisub = new TowelSubs();
  // sorry, unfortunate c++ behavior

  m_CT = loadTable(*m_scene);

  m_multisub->m_towelPtsMsg.fromFiles(m_multisub->m_towelSub.m_names.getCur());
  vector<btVector3> towelPtsCam = toBulletVectors(m_multisub->m_towelPtsMsg.m_data);
  vector<btVector3> towelPtsWorld = m_CT->toWorldFromCamN(towelPtsCam);
  vector<btVector3> towelCorners = toBulletVectors(getCorners(toEigenVectors(towelPtsWorld)));
  BOOST_FOREACH(btVector3& pt, towelCorners) pt += btVector3(.01*METERS,0,0);


  BulletSoftObject::Ptr towel = makeSelfCollidingTowel(towelCorners, m_scene->env->bullet->softBodyWorldInfo, 45, 31);
  m_towel.reset(new TrackedTowel(towel, 45, 31));

  m_scene->env->add(m_towel->m_sim);

}


void TowelVision2::doIteration() {
  
  VectorXf pVis = calcVisibility(m_towel->m_sim->softBody.get(), m_scene->env->bullet->dynamicsWorld, m_CT->worldFromCamUnscaled.getOrigin()*METERS, m_towel->m_nodeInds);

  MatrixXf estFeats = m_towel->featsFromSim();
  
  Eigen::MatrixXf corrEigen = calcCorrProb(estFeats, m_towel->m_sigs, m_obsData.m_obsFeats, pVis, TrackingConfig::outlierParam, m_loglik);
  Eigen::VectorXf inlierFrac = corrEigen.colwise().sum();
  SparseArray corr = toSparseArray(corrEigen, TrackingConfig::cutoff);
  
  m_towel->applyEvidence(corr, m_obsData.m_obsPts);

  updateAllPlots(m_obsData.m_obsPts, m_towel->getNodes(), m_towel->m_sigs, pVis, corr, inlierFrac);

  double start = timeOfDay();
  m_scene->env->step(.03,2,.015);
  cout << timeOfDay() - start << endl;
  m_scene->draw();
  
}

void TowelVision2::beforeIterations() {
  m_obsData.m_obsCloud.reset(new ColorCloud());
  pcl::transformPointCloud(*m_multisub->m_towelPtsMsg.m_data, *m_obsData.m_obsCloud, m_CT->worldFromCamEigen);
  m_obsData.m_obsPts = toBulletVectors(m_obsData.m_obsCloud);
  m_obsData.m_obsFeats = TrackedTowel::featsFromCloud(m_obsData.m_obsCloud);  
  if (TrackingConfig::showKinect)  {
    ColorCloudPtr cloudWorld(new ColorCloud());
    pcl::transformPointCloud(*m_multisub->m_kinectMsg.m_data, *cloudWorld, m_CT->worldFromCamEigen);
    m_kinectPts->setPoints1(cloudWorld);
  }
  else m_kinectPts->clear();
}

void TowelVision2::afterIterations() {
  vector< vector<float> > towelPts = toVecVec(m_towel->getNodes());  
  m_pub.send(VecVecMessage<float>(towelPts));
}
