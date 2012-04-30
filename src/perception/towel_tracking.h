#pragma once
#include "perception/tracking.h"
#include "perception/robot_geometry.h"
#include "robots/pr2.h"
#include "robots/grabbing.h"
#include "simulation/softbodies.h"


struct TrackedTowel : public TrackedObject {
  typedef boost::shared_ptr<TrackedTowel> Ptr;

  BulletSoftObject::Ptr m_sim;
  vector<int> m_nodeInds;
  vector< vector<int> > node2knots;

  vector<float> m_masses;

  TrackedTowel(BulletSoftObject::Ptr, int xres, int yres);
  static Eigen::MatrixXf featsFromCloud(ColorCloudPtr); // x,y,z,a=label
  Eigen::MatrixXf featsFromSim();
  void applyEvidence(const SparseArray& corr, const vector<btVector3>& obsPts); // add forces
  vector<btVector3> getNodes();
  vector<btVector3> getNodes(vector<btVector3>& vel);
  vector<float> getNodeMasses();
  vector<int> getNodeInds();
};

struct TowelTracker2 : public Tracker2 {
  
  struct TowelSubs : public MultiSubscriber {
    CloudMessage m_kinectMsg;        
    CloudMessage m_towelPtsMsg;
    FileSubscriber m_kinectSub;
    FileSubscriber m_towelSub;
    TowelSubs() : m_kinectSub("kinect","pcd"), m_towelSub("towel_pts", "pcd") {
      m_msgs.push_back(&m_kinectMsg);
      m_msgs.push_back(&m_towelPtsMsg);
      m_subs.push_back(&m_kinectSub);
      m_subs.push_back(&m_towelSub);
    }
  };
  TowelSubs* m_multisub;
  
  BulletObject::Ptr m_table;
  TrackedTowel::Ptr m_towel;

  FilePublisher m_pub;
  
  struct ObsData {
    ColorCloudPtr m_obsCloud;
    Eigen::MatrixXf m_obsFeats;
    vector<btVector3> m_obsPts;
  };
  ObsData m_obsData;
  
  // struct EstData {
  //   
  //   m_nodePos;
  //   m_nodeVel;
  //   m_nodeMasses;
  // };
  // EstData m_estData;  
  
  float m_loglik;

  
  TowelTracker2();

  void doIteration(); // correspondence update and physics update
  void beforeIterations(); // e.g. joint updates, coordinate transforms, make depth image
  void afterIterations(); // e.g. send output message, take snapshot
  
};
