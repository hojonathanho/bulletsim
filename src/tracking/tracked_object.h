#pragma once
#include <Eigen/Dense>
#include <simulation/basicobjects.h>
#include <simulation/softbodies.h>
#include <simulation/rope.h>
#include "sparse_utils.h"
#include "clouds/pcl_typedefs.h"
#include "utils/cvmat.h"

class TrackedObject {
public:
  typedef boost::shared_ptr<TrackedObject> Ptr;
  EnvironmentObject::Ptr m_sim;
  std::string m_type;
  int m_nNodes;
  Eigen::VectorXf m_sigs;

  TrackedObject(EnvironmentObject::Ptr sim, string type) : m_sim(sim), m_type(type) {}
  virtual std::vector<btVector3> getPoints() = 0;
  virtual Eigen::MatrixXf getFeatures() = 0;
  virtual void applyEvidence(const SparseMatrixf& corr, const Eigen::MatrixXf& obsPts) = 0;
  virtual EnvironmentObject* getSim()=0;

  static Eigen::MatrixXf featuresTransform(const Eigen::MatrixXf& features) {
  	Eigen::MatrixXf out(features);
  	out.rightCols(3) = colorTransform(out.rightCols(3), CV_BGR2Lab);
//  	Eigen::MatrixXf out(features);
  	return out;
  }

  static Eigen::MatrixXf featuresUntransform(const Eigen::MatrixXf& features) {
  	Eigen::MatrixXf out(features);
  	out.rightCols(3) = colorTransform(out.rightCols(3), CV_Lab2BGR);
//  	Eigen::MatrixXf out(features);
  	return out;
  }

  static Eigen::MatrixXf extractFeatures(ColorCloudPtr in) {
  	Eigen::MatrixXf out(in->size(), 6);
    for (int i=0; i < in->size(); ++i)
    	out.row(i) << in->points[i].x, in->points[i].y, in->points[i].z, in->points[i].b/255.0, in->points[i].g/255.0, in->points[i].r/255.0;
    return featuresTransform(out);
  }

};

class TrackedRope : public TrackedObject { 
public:
  typedef boost::shared_ptr<TrackedRope> Ptr;

  TrackedRope(CapsuleRope::Ptr sim);

  std::vector<btVector3> getPoints();
  Eigen::MatrixXf getFeatures();
  void applyEvidence(const SparseMatrixf& corr, const Eigen::MatrixXf& obsPts);
  CapsuleRope* getSim() {return dynamic_cast<CapsuleRope*>(m_sim.get());}

protected:
  Eigen::VectorXf m_masses;
};

class TrackedTowel : public TrackedObject {
public:
  typedef boost::shared_ptr<TrackedTowel> Ptr;
  TrackedTowel(BulletSoftObject::Ptr, int xres, int yres);
  std::vector<btVector3> getPoints();
  Eigen::MatrixXf getFeatures() {};
  void applyEvidence(const SparseMatrixf& corr, const Eigen::MatrixXf& obsPts); // add forces
  BulletSoftObject* getSim() {return dynamic_cast<BulletSoftObject*>(m_sim.get());};

protected:
  std::vector<int> m_node2vert; // maps node index to bullet vertex index
  std::vector< std::vector<int> > m_vert2nodes; // maps bullet vertex index to indices of nodes
  Eigen::VectorXf m_masses;

};

class TrackedBox : public TrackedObject {
public:
  typedef boost::shared_ptr<TrackedBox> Ptr;

  TrackedBox(BoxObject::Ptr sim);

  std::vector<btVector3> getPoints();
  Eigen::MatrixXf getFeatures() {};
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

BulletSoftObject::Ptr makeTowel(const vector<btVector3>& points, int resolution_x, int resolution_y, btSoftBodyWorldInfo& worldInfo);

