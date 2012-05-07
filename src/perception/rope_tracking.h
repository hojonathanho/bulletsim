#pragma once
#include "perception/tracking.h"
#include "perception/robot_geometry.h"
#include "robots/pr2.h"
#include "robots/grabbing.h"
#include "simulation/rope.h"


struct RopeInitMessage : public Message {
  struct RopeInitData {
    Eigen::VectorXf m_labels;
    Eigen::MatrixXf m_positions;
  };
  RopeInitData m_data;
  RopeInitMessage() : Message() {}
  RopeInitMessage(RopeInitData& data) : Message(), m_data(data) {}
  RopeInitMessage(RopeInitData& data, Json::Value info) : Message(info), m_data(data) {}
  void readDataFrom(fs::path);
  void writeDataTo(fs::path);
};


struct TransformMessage : public Message {
  btTransform m_data;
  void readDataFrom(fs::path);
  void writeDataTo(fs::path);
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


struct RopeHyp {
  typedef boost::shared_ptr<RopeHyp> Ptr;
  float m_loglik;
  float m_age;
  TrackedRope::Ptr m_tracked;
  Environment::Ptr m_env;
  
  RopeHyp(TrackedRope::Ptr tracked, Environment::Ptr env) : m_tracked(tracked), m_env(env), m_age(0) {}
  
};


struct RopeHypWithRobot : public RopeHyp {
  typedef boost::shared_ptr<RopeHypWithRobot> Ptr;
  RaveRobotObject::Ptr m_robot;
  boost::shared_ptr<MonitorForGrabbing> m_lMonitor, m_rMonitor;  
    
  RopeHypWithRobot(TrackedRope::Ptr tracked, Environment::Ptr env, RaveRobotObject::Ptr pr2);
  void step();
};

struct RopeSubs : public MultiSubscriber {
  CloudMessage m_kinectMsg;        
  CloudMessage m_ropePtsMsg;

  FileSubscriber m_kinectSub;
  FileSubscriber m_ropeSub;
  RopeSubs() : m_kinectSub("kinect","pcd"), m_ropeSub("rope_pts", "pcd") {
    m_msgs.push_back(&m_kinectMsg);
    m_msgs.push_back(&m_ropePtsMsg);

    m_subs.push_back(&m_kinectSub);
    m_subs.push_back(&m_ropeSub);
  }
};

struct RopeSubs2 : public RopeSubs {

  ImageMessage m_labelMsg;
  FileSubscriber m_labelSub;
  RopeSubs2() : RopeSubs(), m_labelSub("labels","bmp") {
    m_msgs.push_back(&m_labelMsg);
    m_subs.push_back(&m_labelSub);
  }
};




struct SingleHypRopeTracker : public Tracker2 {
  SingleHypRopeTracker();

  RopeSubs* m_multisub;

  RopeInitMessage m_ropeInitMsg;
  FileSubscriber m_ropeInitSub;
  
  RopeHyp::Ptr m_hyp;
  
  BulletObject::Ptr m_table;
  ColorCloudPtr m_obsCloud;
  Eigen::MatrixXf m_obsFeats;
  Eigen::MatrixXf m_depthImage;
  cv::Mat m_ropeMask;


  struct ObsData {
    ColorCloudPtr m_obsCloud;
    Eigen::MatrixXf m_obsFeats;
    vector<btVector3> m_obsPts;
  };
  ObsData m_obsData;

  FilePublisher m_pub;

  virtual void doIteration();
  virtual void beforeIterations();
  virtual void afterIterations();
};

struct DefaultSingleHypRopeTracker : public SingleHypRopeTracker {
  RopeSubs* m_multisub;
  DefaultSingleHypRopeTracker();
  virtual void beforeIterations();
};

struct SingleHypRobotAndRopeTracker : public SingleHypRopeTracker {
  SingleHypRobotAndRopeTracker();
  
  KinectTransformer* m_kinectTrans;
  FileSubscriber m_jointSub;
  FileSubscriber m_basePoseSub;
  Retimer<VectorMessage<double> > m_retimer;
  Retimer<TransformMessage> m_retimer2;
  
  RopeHypWithRobot::Ptr m_hyp;
  
  boost::shared_ptr<PR2Manager> m_pr2m;
  
  virtual void doIteration();
  virtual void beforeIterations();
  virtual void afterIterations();

};

struct DefaultSingleHypRobotAndRopeTracker : public SingleHypRobotAndRopeTracker {
  DefaultSingleHypRobotAndRopeTracker();
};

/*

struct MultiHypRopeTracker : public Tracker2 {

  MultiHypRopeTracker();

  RopeSubs* m_multisub;

  RopeInitMessage m_ropeInitMsg;
  FileSubscriber m_ropeInitSub;
  
  std::vector<RopeHyp::Ptr> m_hyps;
  
  BulletObject::Ptr m_table;
  ColorCloudPtr m_obsCloud;
  Eigen::MatrixXf m_obsFeats;
  Eigen::MatrixXf m_depthImage;
  cv::Mat m_ropeMask;

  FilePublisher m_pub;

  virtual void doIteration();
  virtual void beforeIterations();
  virtual void afterIterations();

  void addHyp(TrackedRope::Ptr rope);
  void removeHyp(int);
  void cullHyps();


};



struct MultiHypRobotAndRopeTracker : public MultiHypRopeTracker {
  MultiHypRobotAndRopeTracker();
  
  KinectTrans* m_kinectTrans;
  FileSubscriber m_jointSub;
  Retimer<VectorMessage<double> > m_retimer;
  
  boost::shared_ptr<PR2Manager> m_pr2m; // todo: make a class MultiPR2Manager
  
  virtual void doIteration();
  virtual void beforeIterations();
  virtual void afterIterations();


};

*/
