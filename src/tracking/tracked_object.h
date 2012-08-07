#pragma once
#include <Eigen/Dense>
#include <simulation/basicobjects.h>
#include <simulation/softbodies.h>
#include <simulation/rope.h>
#include "sparse_utils.h"
#include "clouds/pcl_typedefs.h"
#include "utils/cvmat.h"
#include "utils_tracking.h"
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "clouds/utils_pcl.h"
#include "tracking_defs.h"

class TrackedObject {
public:
  typedef boost::shared_ptr<TrackedObject> Ptr;
  EnvironmentObject::Ptr m_sim;
  std::string m_type;
  int m_nNodes;
  Eigen::VectorXf m_sigs;
  std::vector<FeatureType> m_featureTypes;
  int m_featureDim;
  Eigen::MatrixXf m_features;
  Eigen::MatrixXf m_colors;

  TrackedObject(EnvironmentObject::Ptr sim, string type);

  void setFeatureTypes(const std::vector<FeatureType>&);
  virtual std::vector<btVector3> getPoints() = 0;
  virtual Eigen::MatrixXf& getColors();
  
  void updateFeatures();
  Eigen::MatrixXf& getFeatures();
  virtual Eigen::VectorXf getPriorDist() = 0;
  virtual Eigen::VectorXf getOutlierDist() = 0;
  virtual void applyEvidence(const SparseMatrixf& corr, const Eigen::MatrixXf& obsPts) = 0;
  virtual EnvironmentObject* getSim()=0;

};

class TrackedRope : public TrackedObject { 
public:
  typedef boost::shared_ptr<TrackedRope> Ptr;

  TrackedRope(CapsuleRope::Ptr sim);

  std::vector<btVector3> getPoints();
  Eigen::VectorXf getPriorDist();
  Eigen::VectorXf getOutlierDist();
  void applyEvidence(const SparseMatrixf& corr, const Eigen::MatrixXf& obsPts);
  CapsuleRope* getSim() {return dynamic_cast<CapsuleRope*>(m_sim.get());}
  cv::Mat makeTexture(ColorCloudPtr cloud);
  void setColorsFromTexture();

protected:
  Eigen::VectorXf m_masses;
};

class TrackedTowel : public TrackedObject {
public:
  typedef boost::shared_ptr<TrackedTowel> Ptr;
  TrackedTowel(BulletSoftObject::Ptr, int xres, int yres);
  std::vector<btVector3> getPoints();
  std::vector<btVector3> getNormals();
  Eigen::VectorXf getPriorDist();
  Eigen::VectorXf getOutlierDist();
  void applyEvidence(const SparseMatrixf& corr, const Eigen::MatrixXf& obsPts); // add forces
  BulletSoftObject* getSim() {return dynamic_cast<BulletSoftObject*>(m_sim.get());};
  cv::Mat makeTexture(const vector<btVector3>& corners, cv::Mat image, CoordinateTransformer* transformer);
  cv::Mat makeTexturePC(ColorCloudPtr cloud);
  void setColorsFromTexture();
  

protected:
  std::vector<int> m_node2vert; // maps node index to bullet vertex index
  std::vector< std::vector<int> > m_vert2nodes; // maps bullet vertex index to indices of nodes
  std::vector<int> m_vert2tex; // maps bullet vertex index to texture coordinate index
  std::vector< std::vector<int> > m_face2verts;
  Eigen::VectorXf m_masses;

};

class TrackedBox : public TrackedObject {
public:
  typedef boost::shared_ptr<TrackedBox> Ptr;

  TrackedBox(BoxObject::Ptr sim);

  std::vector<btVector3> getPoints();
  Eigen::MatrixXf extractFeatures(ColorCloudPtr in);
  Eigen::VectorXf getPriorDist();
  Eigen::VectorXf getOutlierDist();
  void applyEvidence(const SparseMatrixf& corr, const Eigen::MatrixXf& obsPts);
  BoxObject* getSim() {return dynamic_cast<BoxObject*>(m_sim.get());}

protected:
  int m_nEdgeNodesX;
  int m_nEdgeNodesY;
  int m_nEdgeNodesZ;
  Eigen::VectorXf m_masses;
};

std::vector<btVector3> calcImpulsesDamped(const std::vector<btVector3>& estPos, const std::vector<btVector3>& estVel, 
    const std::vector<btVector3>& obsPts, const SparseMatrixf& corr, const vector<float>& masses, float kp, float kd);

BulletSoftObject::Ptr makeTowel(const vector<btVector3>& points, int resolution_x, int resolution_y);

