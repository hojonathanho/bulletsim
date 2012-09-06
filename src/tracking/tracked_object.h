#pragma once
#include <Eigen/Dense>
#include <simulation/basicobjects.h>
#include <simulation/softbodies.h>
#include <simulation/rope.h>
#include "sparse_utils.h"
#include "clouds/pcl_typedefs.h"
#include "utils/cvmat.h"
#include "config_tracking.h"
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "clouds/utils_pcl.h"

class CoordinateTransformer;

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

class TrackedCloth : public TrackedObject {
public:
  typedef boost::shared_ptr<TrackedCloth> Ptr;

  TrackedCloth(BulletSoftObject::Ptr, cv::Mat image, int xres, int yres, float sx, float sy);

  cv::Point2f getTexCoord(const int& nodeIdx);

  std::vector<btVector3> getPoints();
  std::vector<btVector3> getNormals();
  const Eigen::VectorXf getPriorDist();
  void applyEvidence(const Eigen::MatrixXf& corr, const Eigen::MatrixXf& obsPts); // add forces
  BulletSoftObject* getSim() {return dynamic_cast<BulletSoftObject*>(m_sim.get());};
  void initColors();
  float m_sx, m_sy; //towel dimensions

protected:
  std::vector<int> m_node2vert; // maps node index to bullet vertex index
  std::vector< std::vector<int> > m_vert2nodes; // maps bullet vertex index to indices of nodes
  std::vector<int> m_vert2tex; // maps bullet vertex index to texture coordinate index
  std::vector< std::vector<int> > m_face2verts;
  Eigen::VectorXf m_masses;
};

class TrackedTowel : public TrackedObject {
public:
  typedef boost::shared_ptr<TrackedTowel> Ptr;

  TrackedTowel(BulletSoftObject::Ptr, int xres, int yres, float sx, float sy);

  std::vector<btVector3> getPoints();
  std::vector<btVector3> getNormals();
  const Eigen::VectorXf getPriorDist();
  void applyEvidence(const Eigen::MatrixXf& corr, const Eigen::MatrixXf& obsPts); // add forces
  BulletSoftObject* getSim() {return dynamic_cast<BulletSoftObject*>(m_sim.get());};
  cv::Mat makeTexture(const vector<btVector3>& corners, cv::Mat image, CoordinateTransformer* transformer);
  cv::Point2f textureCoordinate (int node_id);
  void initColors();
  float m_sx, m_sy; //towel dimensions

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

BulletSoftObject::Ptr makeCloth(const vector<btVector3>& points, int resolution_x, int resolution_y, float mass);
BulletSoftObject::Ptr makeTowel(const vector<btVector3>& points, int resolution_x, int resolution_y, float mass);

// Find the 3d corners of the polygon. First, it finds the corners in the 2d mask. The 3d corner is given by the
// intersection between the ray (from camera origin through pixel) and the plane (containing the point cloud)
std::vector<btVector3> polyCorners(ColorCloudPtr cloud, cv::Mat mask, CoordinateTransformer* transformer);


// Assumes corners are in the xy-plane and the thickness is the z-axis direction
// good max_tet_vol: 0.0008*METERS*METERS*METERS
BulletSoftObject::Ptr makeSponge(const std::vector<btVector3>& corners, float thickness, float max_tet_vol, float mass);

// Returns the approximate polygon of the concave hull of the cloud
// The points are being projected to the xy plane
std::vector<btVector3> polyCorners(ColorCloudPtr cloud);
