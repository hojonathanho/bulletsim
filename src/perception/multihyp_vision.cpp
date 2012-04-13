#include "multihyp_vision.h"
using namespace std;
using namespace Eigen;
#include "perception/utils_perception.h"
#include "perception/config_perception.h"
#include "perception/optimization_forces.h"
#include <pcl/common/transforms.h>
#include "perception/get_nodes.h"
#include "perception/visibility.h"

static osg::Vec4f hyp_colors[6] = {osg::Vec4f(1,0,0,.3),
			    osg::Vec4f(0,1,0,.3),
			    osg::Vec4f(0,0,1,.3),
			    osg::Vec4f(1,1,0,.3),
			    osg::Vec4f(0,1,1,.3),
			    osg::Vec4f(1,0,1,.3)
};
static int COLOR_IND=0;
static osg::Vec4f getColor() {
  COLOR_IND = (COLOR_IND + 1) % 6;
  return hyp_colors[COLOR_IND];
}



MultiHypRopeVision::MultiHypRopeVision() : Vision2(), m_ropeInitMsg(), m_ropeInitSub("init_rope","txt"), m_pub("rope_model","txt") {
  cout << "MultiHypRopeVision::MultiHypRopeVision()" << endl;
  Vision2::m_multisub = m_multisub = new RopeSubs();

  m_CT = loadTable(*m_scene);

}



void MultiHypRopeVision::beforeIterations() {
  ColorCloudPtr cloudCam  = m_multisub->m_kinectMsg.m_data;
  ColorCloudPtr cloudWorld(new ColorCloud());
  pcl::transformPointCloud(*cloudCam, *cloudWorld, m_CT->worldFromCamEigen);
  if (TrackingConfig::showKinect) m_kinectPts->setPoints1(cloudWorld);
  else {m_kinectPts->clear();}
  cv::Mat labels = m_multisub->m_labelMsg.m_data;
  m_ropeMask = labels < 2;
  m_depthImage = getDepthImage(cloudCam);
  pcl::transformPointCloud(*m_multisub->m_ropePtsMsg.m_data, *m_obsCloud, m_CT->worldFromCamEigen);
  m_obsFeats = TrackedRope::featsFromCloud(m_obsCloud);


  bool gotRope = m_ropeInitSub.recv(m_ropeInitMsg,false);
  if (gotRope) {
    TrackedRope::Ptr trackedRope(new TrackedRope(m_ropeInitMsg, m_CT));
    osg::Vec4f color = getColor();
    BOOST_FOREACH(BulletObject::Ptr bo, trackedRope->m_sim->children) bo->setColor(color[0], color[1], color[2], color[3]);
    addHyp(trackedRope);
  }
}


void MultiHypRopeVision::doIteration() {
  for (int i=0; i < m_hyps.size(); i++) {
    TrackedRope::Ptr hyp = m_hyps[i]->m_tracked;
    MatrixXf ropePtsCam  = toEigenMatrix(m_CT->toCamFromWorldN(getNodes(hyp->m_sim)));
    VectorXf pVis  = calcVisibility(ropePtsCam, m_depthImage, m_ropeMask); 
    Eigen::MatrixXf corrEigen = calcCorrProb(hyp->featsFromSim(), hyp->m_sigs, m_obsFeats, pVis, TrackingConfig::outlierParam, hyp->m_loglik);
    SparseArray corr = toSparseArray(corrEigen, TrackingConfig::cutoff);
    hyp->applyEvidence(corr, toEigenMatrix(m_obsCloud));
    
    
    if (i==0) {      
      
      vector<btVector3> obsPts = toBulletVectors(m_obsCloud);
      vector<btVector3> estPts = getNodes(hyp->m_sim);
      Eigen::VectorXf inlierFrac = corrEigen.colwise().sum();
      updateAllPlots(obsPts, estPts, hyp->m_sigs, pVis, corr, inlierFrac);

    }
    
    m_hyps[i]->m_fork->env->step(.03,2,.015);
  }

  m_scene->draw();

}

void MultiHypRopeVision::afterIterations() {
  vector< vector<float> > vv; 
  m_pub.send(VecVecMessage<float>(vv));
  BOOST_FOREACH(RopeHyp::Ptr hyp, m_hyps) hyp->m_age++;
  cullHyps();
}

void MultiHypRopeVision::cullHyps() {  
  // TODO
  // if (m_hyps.size() > 1 && m_hyps[0]->m_age >= MIN_CULL_AGE && m_ropeHypoths[1]->m_age >= MIN_CULL_AGE) {
  //   cout << "log likelihoods: " << m_ropeHypoths[0]->m_loglik << " " << m_ropeHypoths[1]->m_loglik << endl;
  //   if (m_ropeHypoths[0]->m_loglik > .95*m_ropeHypoths[1]->m_loglik) {
  //     cout << "removing alternate hypoth" << endl;
  //     removeRopeHypoth(m_ropeHypoths[1]);
  //   }
  //   else {
  //     cout << "removing original hypoth" << endl;
  //     removeRopeHypoth(m_ropeHypoths[0]);
  //   }
  // }
}

void MultiHypRopeVision::addHyp(TrackedRope::Ptr tr) {

  BulletInstance::Ptr bullet2(new BulletInstance);
  bullet2->setGravity(BulletConfig::gravity);
  OSGInstance::Ptr osg2(new OSGInstance);
  m_scene->osg->root->addChild(osg2->root.get());
  Fork::Ptr fork(new Fork(m_scene->env, bullet2, osg2));
  fork->env->add(tr->m_sim);
  m_hyps.push_back(RopeHyp::Ptr(new RopeHyp(tr, fork)));

}

void MultiHypRopeVision::removeHyp(int i) {
  RopeHyp::Ptr hyp = m_hyps[i];
  m_scene->osg->root->removeChild(hyp->m_fork->env->osg->root.get());
  m_hyps.erase(m_hyps.begin() + i, m_hyps.begin() + i + 1);
}


MultiHypRobotAndRopeVision::MultiHypRobotAndRopeVision() : MultiHypRopeVision() {
}
  

void MultiHypRobotAndRopeVision::doIteration() {}
void MultiHypRobotAndRopeVision::beforeIterations() {}
void MultiHypRobotAndRopeVision::afterIterations() {}

