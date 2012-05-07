#include "towel_tracking.h"
using namespace std;
using namespace Eigen;
#include "perception/utils_perception.h"
#include "perception/config_perception.h"
#include "perception/optimization_forces.h"
#include <pcl/common/transforms.h>
#include "perception/get_nodes.h"
#include "perception/visibility.h"
#include "simulation/bullet_io.h"
#include "robots/ros2rave.h"
#include "perception/make_bodies.h"
#include "utils/vector_io.h"
#include "clouds/geom.h"
#include "perception/apply_impulses.h"

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
    ColorPoint& pt = cloud->points[i];
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



TowelTracker2::TowelTracker2() : m_pub("towel_model", "txt") {

  Tracker2::m_multisub = m_multisub = new TowelSubs();
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


void TowelTracker2::doIteration() {
  
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

void TowelTracker2::beforeIterations() {
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

void TowelTracker2::afterIterations() {
  vector< vector<float> > towelPts = toVecVec(m_towel->getNodes());  
  m_pub.send(VecVecMessage<float>(towelPts));
}
