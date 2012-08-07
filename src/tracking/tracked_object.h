#pragma once
#include <Eigen/Dense>
#include <simulation/basicobjects.h>
#include <simulation/softbodies.h>
#include <simulation/rope.h>
#include "sparse_utils.h"
#include "clouds/pcl_typedefs.h"
#include "utils/cvmat.h"
#include "config_tracking.h"
#include "utils_tracking.h"
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "clouds/utils_pcl.h"

class TrackedObject {
public:
  typedef boost::shared_ptr<TrackedObject> Ptr;
  EnvironmentObject::Ptr m_sim;
  std::string m_type;
  int m_nNodes;
  Eigen::VectorXf m_sigs;
  Eigen::MatrixXf m_colors;

  TrackedObject(EnvironmentObject::Ptr sim, string type);
  void init();

  virtual std::vector<btVector3> getPoints() = 0;
  Eigen::MatrixXf& getColors();
  virtual void initColors() { m_colors = Eigen::MatrixXf::Constant(m_nNodes, 3, 1.0); }

  virtual const Eigen::VectorXf getPriorDist() = 0;
  virtual const Eigen::VectorXf getOutlierDist() { return Eigen::VectorXf::Constant(3, TrackingConfig::pointOutlierDist*METERS); }
  virtual const Eigen::VectorXf getOutlierStdev() { return Eigen::VectorXf::Constant(3, TrackingConfig::pointPriorDist*METERS); }
  virtual void applyEvidence(const Eigen::MatrixXf& corr, const Eigen::MatrixXf& obsPts) = 0;
  virtual EnvironmentObject* getSim()=0;
};

class TrackedRope : public TrackedObject { 
public:
  typedef boost::shared_ptr<TrackedRope> Ptr;

  TrackedRope(CapsuleRope::Ptr sim);

  std::vector<btVector3> getPoints();
  const Eigen::VectorXf getPriorDist();
  void applyEvidence(const Eigen::MatrixXf& corr, const Eigen::MatrixXf& obsPts);
  CapsuleRope* getSim() {return dynamic_cast<CapsuleRope*>(m_sim.get());}
  cv::Mat makeTexture(ColorCloudPtr cloud);
  void initColors();

protected:
  Eigen::VectorXf m_masses;
};

class TrackedTowel : public TrackedObject {
public:
  typedef boost::shared_ptr<TrackedTowel> Ptr;

  TrackedTowel(BulletSoftObject::Ptr, int xres, int yres);

  std::vector<btVector3> getPoints();
  std::vector<btVector3> getNormals();
  const Eigen::VectorXf getPriorDist();
  void applyEvidence(const Eigen::MatrixXf& corr, const Eigen::MatrixXf& obsPts); // add forces
  BulletSoftObject* getSim() {return dynamic_cast<BulletSoftObject*>(m_sim.get());};
  cv::Mat makeTexture(const vector<btVector3>& corners, cv::Mat image, CoordinateTransformer* transformer);
  void initColors();
  
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
  const Eigen::VectorXf getPriorDist();
  void applyEvidence(const Eigen::MatrixXf& corr, const Eigen::MatrixXf& obsPts);
  BoxObject* getSim() {return dynamic_cast<BoxObject*>(m_sim.get());}

protected:
  int m_nEdgeNodesX;
  int m_nEdgeNodesY;
  int m_nEdgeNodesZ;
  Eigen::VectorXf m_masses;
};

std::vector<btVector3> calcImpulsesDamped(const std::vector<btVector3>& estPos, const std::vector<btVector3>& estVel,
  const std::vector<btVector3>& obsPts, const Eigen::MatrixXf& corr, const vector<float>& masses, float kp, float kd) ;

BulletSoftObject::Ptr makeTowel(const vector<btVector3>& points, float node_density, float surface_density, int& resolution_x, int& resolution_y);
