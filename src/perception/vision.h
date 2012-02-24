#pragma once
#include "simulation/simplescene.h"
#include "simulation/recording.h"
#include "simulation/softbodies.h"
#include "comm/comm.h"
#include "perception/utils_perception.h"
#include <vector>
#include "clouds/comm_pcl.h"
#include "clouds/comm_cv.h"
#include "simulation/rope.h"

using namespace std; 
const int BIGINT = 9999999;

struct Vision {
    
  Scene* m_scene;    
  CoordinateTransformer* m_CT;

  PointCloudPlot::Ptr m_kinectPts;
  PlotSpheres::Ptr m_estPlot;
  PointCloudPlot::Ptr m_obsPlot;
  PlotLines::Ptr m_corrLines;
    
  std::vector<Message*> m_msgs;
  CloudMessage m_kinectMsg;
  std::vector<FileSubscriber*> m_subs;
  FilePublisher* m_pub;
  std::vector<bool> m_gotEm;
    
  ScreenRecorder* m_rec;
    
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
    
};

struct TowelVision : public Vision {

  TowelVision();

  BulletSoftObject::Ptr m_towel;
  Eigen::VectorXf m_sigs;
  BulletObject::Ptr m_table;
  CloudMessage m_towelPtsMsg;
  vector<btVector3> m_towelObsPts;
  vector<btVector3> m_towelEstPts;
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
  vector<btVector3> m_ropeObsPts;
  vector<btVector3> m_ropeEstPts;
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
