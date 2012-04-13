#pragma once
#include "vision.h"

struct RopeHyp {
  typedef boost::shared_ptr<RopeHyp> Ptr;
  float m_loglik;
  float m_age;
  TrackedRope::Ptr m_tracked;
  Fork::Ptr m_fork;
  
  RopeHyp(TrackedRope::Ptr tracked, Fork::Ptr fork) : m_tracked(tracked), m_fork(fork) {}
  
};


struct MultiHypRopeVision : public Vision2 {
  MultiHypRopeVision();

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
      m_subs.push_back(&m_kinectSub);
      m_subs.push_back(&m_ropeSub);
      m_subs.push_back(&m_labelSub);
    }
  };
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
  

  virtual void doIteration();
  virtual void beforeIterations();
  virtual void afterIterations();


};
