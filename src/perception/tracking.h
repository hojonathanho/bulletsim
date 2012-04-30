#pragma once
#include "simulation/simplescene.h"
#include "comm/comm.h"
#include "perception/utils_perception.h"
#include "clouds/comm_pcl.h"
#include "clouds/comm_cv.h"
#include "perception/plotting_perception.h"




struct TrackedObject {
  typedef boost::shared_ptr<TrackedObject> Ptr;
  
  TrackedObject() : m_age(0) {}
  

  Eigen::VectorXf m_sigs;
  float m_loglik; // DEPRECATED: let Tracker class keep track of likelihood of fork
  int m_age; // DEPRECATED?

  virtual Eigen::MatrixXf featsFromSim() = 0;
  virtual void applyEvidence(const SparseArray& corr, const Eigen::MatrixXf& obsPts) {}
  virtual void applyEvidence(const SparseArray& corr, const vector<btVector3>& obsPts) {}  
};


struct Tracker2 {

  // simulation
  CoordinateTransformer* m_CT;
  Scene* m_scene;

  // plots
  PointCloudPlot::Ptr m_kinectPts;
  PlotSpheres::Ptr m_estPlot;
  PointCloudPlot::Ptr m_obsPlot;
  PlotLines::Ptr m_corrLines;
    
  MultiSubscriber* m_multisub;  
    
  Tracker2();
  
  
  void runOnline(); // update loop. when live, iterates until next message
  void runOffline(); 
  virtual void doIteration() = 0; // correspondence update and physics update
  virtual void beforeIterations() = 0; // e.g. joint updates, coordinate transforms, make depth image
  virtual void afterIterations() = 0; // e.g. send output message, take snapshot
    
  void updateAllPlots(const std::vector<btVector3>& obsPts, const std::vector<btVector3>& estPts, 
    const Eigen::VectorXf& sigs, const Eigen::VectorXf& pVis, const SparseArray& corr, 
    const Eigen::VectorXf& inlierFrac);


  virtual void makeScene(); // may want to use some fancier type of scene
  virtual Scene* getScene(); // you can use covariant return types with this
};



CoordinateTransformer* loadTable(Scene& scene); // load table from standard location and add it to the scene
