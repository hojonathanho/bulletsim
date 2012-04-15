#pragma once
#include "perception/vision.h"
#include "perception/robot_geometry.h"
#include "robots/pr2.h"
#include "robots/grabbing.h"

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
  ImageMessage m_labelMsg;

  FileSubscriber m_kinectSub;
  FileSubscriber m_ropeSub;
  FileSubscriber m_labelSub;
  RopeSubs() : m_kinectSub("kinect","pcd"), m_ropeSub("rope_pts", "pcd"), m_labelSub("labels","bmp") {
    m_msgs.push_back(&m_kinectMsg);
    m_msgs.push_back(&m_ropePtsMsg);
    m_msgs.push_back(&m_labelMsg);

    m_subs.push_back(&m_kinectSub);
    m_subs.push_back(&m_ropeSub);
    m_subs.push_back(&m_labelSub);
  }
};


struct SingleHypRopeVision : public Vision2 {
  SingleHypRopeVision();

  RopeSubs* m_multisub;

  RopeInitMessage m_ropeInitMsg;
  FileSubscriber m_ropeInitSub;
  
  RopeHyp::Ptr m_hyp;
  
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
};



struct SingleHypRobotAndRopeVision : public SingleHypRopeVision {
  SingleHypRobotAndRopeVision();
  
  KinectTrans* m_kinectTrans;
  FileSubscriber m_jointSub;
  Retimer<VectorMessage<double> > m_retimer;
  
  RopeHypWithRobot::Ptr m_hyp;
  
  boost::shared_ptr<PR2Manager> m_pr2m;
  
  virtual void doIteration();
  virtual void beforeIterations();
  virtual void afterIterations();


};


/*

struct MultiHypRopeVision : public Vision2 {

  MultiHypRopeVision();

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



struct MultiHypRobotAndRopeVision : public MultiHypRopeVision {
  MultiHypRobotAndRopeVision();
  
  KinectTrans* m_kinectTrans;
  FileSubscriber m_jointSub;
  Retimer<VectorMessage<double> > m_retimer;
  
  boost::shared_ptr<PR2Manager> m_pr2m; // todo: make a class MultiPR2Manager
  
  virtual void doIteration();
  virtual void beforeIterations();
  virtual void afterIterations();


};

*/
