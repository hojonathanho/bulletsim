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
  
  Eigen::VectorXf m_sigs;
  virtual Eigen::MatrixXf featsFromSim() = 0;
  virtual void applyEvidence(const SparseArray& corr, const Eigen::MatrixXf& obsPts) = 0; // add forces, update loglik
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

class History {
public:
  History(int maxsize=10);
  void put(float x);
  float sum();
  int size();
  bool full();
  
  int m_maxsize;    

private:
  std::queue<float> m_data;
  float m_sum;
};

struct TrackedRope : public TrackedObject { // TODO: visibility
  typedef boost::shared_ptr<TrackedRope> Ptr;

  CapsuleRope::Ptr m_sim;
  Eigen::VectorXf m_labels;
  History m_forceHistory;

  TrackedRope(const RopeInitMessage&, CoordinateTransformer*);
  static Eigen::MatrixXf featsFromCloud(ColorCloudPtr); // x,y,z,a=label
  Eigen::MatrixXf featsFromSim();
  void applyEvidence(const SparseArray& corr, const Eigen::MatrixXf& obsPts); // add forces
protected:
  void init(const std::vector<btVector3>& nodes, const Eigen::VectorXf& labels);
};

struct TrackedTowel : public TrackedObject {
  TrackedTowel(const std::vector< std::vector<float> > corners);
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
