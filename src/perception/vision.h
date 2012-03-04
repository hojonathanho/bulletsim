#pragma once
#include "simulation/simplescene.h"
#include "simulation/recording.h"
#include "simulation/softbodies.h"
#include "comm/comm.h"
#include "perception/utils_perception.h"
#include <vector>
#include <queue>
#include "clouds/comm_pcl.h"
#include "clouds/comm_cv.h"
#include "simulation/rope.h"
#include "perception/plotting_perception.h"

const int BIGINT = 9999999;

struct Vision {
    

  // simulation
  Scene* m_scene;    
  CoordinateTransformer* m_CT;

  // plots
  PointCloudPlot::Ptr m_kinectPts;
  PlotSpheres::Ptr m_estPlot;
  PointCloudPlot::Ptr m_obsPlot;
  PlotLines::Ptr m_corrLines;
    
  // comm
  std::vector<Message*> m_msgs;
  CloudMessage m_kinectMsg;
  std::vector<FileSubscriber*> m_subs;
  FilePublisher* m_pub;
  std::vector<bool> m_gotEm;
    
  Vision();
  ~Vision();
  void runOnline(); // update loop. when live, iterates until next message
  void runOffline(); 
  virtual void doIteration() = 0; // correspondence update and physics update
  virtual void beforeIterations() = 0; // e.g. joint updates, coordinate transforms, make depth image
  virtual void afterIterations() = 0; // e.g. send output message, take snapshot
    
  virtual void setupComm(); // create subs, pub, msgs
  virtual void setupScene(); // create scene, add objects to it, recorder
  bool recvAll(); // recv all messages that haven't been received
  
  void updateAllPlots(const std::vector<btVector3>& obsPts, const std::vector<btVector3>& estPts, const Eigen::VectorXf& sigs, const Eigen::VectorXf& pVis, const SparseArray& corr, const Eigen::VectorXf& inlierFrac);
    
};


struct TowelVision : public Vision {

  TowelVision();

  BulletSoftObject::Ptr m_towel;
  Eigen::VectorXf m_sigs;
  BulletObject::Ptr m_table;
  CloudMessage m_towelPtsMsg;
  std::vector<btVector3> m_towelObsPts;
  std::vector<btVector3> m_towelEstPts;
  FileSubscriber* m_towelSub;

  void doIteration();
  void beforeIterations();
  void afterIterations();

  void setupComm();
  void setupScene();

};

struct RopeVision : public Vision {
  RopeVision();

  CapsuleRope::Ptr m_rope;
  Eigen::VectorXf m_sigs;
  BulletObject::Ptr m_table;
  std::vector<btVector3> m_ropeObsPts;
  std::vector<btVector3> m_ropeEstPts;
  FileSubscriber* m_ropeSub;
  CloudMessage m_ropePtsMsg;
  FileSubscriber* m_labelSub;
  ImageMessage m_labelMsg;

  Eigen::MatrixXf m_depthImage;
  cv::Mat m_ropeMask;

  void doIteration();
  void beforeIterations();
  void afterIterations();

  void setupComm();
  void setupScene();


};

struct TrackedObject {
  typedef boost::shared_ptr<TrackedObject> Ptr;
  
  TrackedObject();

  Eigen::VectorXf m_sigs;
  float m_loglik;
  int m_age;

  virtual Eigen::MatrixXf featsFromSim() = 0;
  virtual void applyEvidence(const SparseArray& corr, const Eigen::MatrixXf& obsPts) = 0;
};


struct RopeInitMessage : public Message {
  struct RopeInitData {
    Eigen::VectorXf m_labels;
    Eigen::MatrixXf m_positions;
  };
  RopeInitData m_data;
  RopeInitMessage() : Message() {}
  RopeInitMessage(RopeInitData& data) : Message(), m_data(data) {}
  RopeInitMessage(RopeInitData& data, Value info) : Message(info), m_data(data) {}
  void readDataFrom(path);
  void writeDataTo(path);
};

struct TrackedRope : public TrackedObject { // TODO: visibility
  typedef boost::shared_ptr<TrackedRope> Ptr;

  CapsuleRope::Ptr m_sim;
  Eigen::VectorXf m_labels;

  TrackedRope(const RopeInitMessage&, CoordinateTransformer*);
  static Eigen::MatrixXf featsFromCloud(ColorCloudPtr); // x,y,z,a=label
  Eigen::MatrixXf featsFromSim();
  void applyEvidence(const SparseArray& corr, const Eigen::MatrixXf& obsPts); // add forces
protected:
  void init(const std::vector<btVector3>& nodes, const Eigen::VectorXf& labels);
};

struct RopeVision2 : public Vision {
  RopeVision2();
  std::vector<TrackedRope::Ptr> m_ropeHypoths;
  std::map<TrackedRope::Ptr, Fork::Ptr> m_rope2fork;
  BulletObject::Ptr m_table;
  ColorCloudPtr m_obsCloud;
  Eigen::MatrixXf m_obsFeats;
  FileSubscriber* m_ropeInitSub;
  RopeInitMessage m_ropeInitMsg;
  FileSubscriber* m_ropeSub;
  CloudMessage m_ropePtsMsg;
  FileSubscriber* m_labelSub;
  ImageMessage m_labelMsg;
  Eigen::MatrixXf m_depthImage;
  cv::Mat m_ropeMask;

  void doIteration();
  void beforeIterations();
  void afterIterations();

  void setupComm();
  void setupScene();

  void addRopeHypoth(TrackedRope::Ptr rope);
  void removeRopeHypoth(TrackedRope::Ptr rope);
  void cullHypoths();


};


struct TrackedObject2 {
  typedef boost::shared_ptr<TrackedObject> Ptr;

  Eigen::VectorXf m_sigs;

  TrackedObject2() {}
  ~TrackedObject2() {}  


  virtual Eigen::MatrixXf featsFromSim() = 0;
  virtual void applyEvidence(const SparseArray& corr, const vector<btVector3>& obsPts) = 0;

};

struct TrackedTowel : public TrackedObject2 {
  typedef boost::shared_ptr<TrackedTowel> Ptr;

  BulletSoftObject::Ptr m_sim;
  vector<int> m_nodeInds;
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

struct Vision2 {

  // simulation
  CoordinateTransformer* m_CT;
  Scene* m_scene;

  // plots
  PointCloudPlot::Ptr m_kinectPts;
  PlotSpheres::Ptr m_estPlot;
  PointCloudPlot::Ptr m_obsPlot;
  PlotLines::Ptr m_corrLines;
    
  MultiSubscriber m_multisub;  
  FilePublisher m_pub;
    
  Vision2();
  ~Vision2();
  void runOnline(); // update loop. when live, iterates until next message
  void runOffline(); 
  virtual void doIteration() = 0; // correspondence update and physics update
  virtual void beforeIterations() = 0; // e.g. joint updates, coordinate transforms, make depth image
  virtual void afterIterations() = 0; // e.g. send output message, take snapshot
    
  void updateAllPlots(const std::vector<btVector3>& obsPts, const std::vector<btVector3>& estPts, const Eigen::VectorXf& sigs, const Eigen::VectorXf& pVis, const SparseArray& corr, const Eigen::VectorXf& inlierFrac);


};


struct TowelVision2 : public Vision2 {
  
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
  
  BulletObject::Ptr m_table;
  TrackedTowel::Ptr m_towel;
  TowelSubs m_multisub;
  
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

  
  TowelVision2();

  void doIteration(); // correspondence update and physics update
  void beforeIterations(); // e.g. joint updates, coordinate transforms, make depth image
  void afterIterations(); // e.g. send output message, take snapshot
  
};

