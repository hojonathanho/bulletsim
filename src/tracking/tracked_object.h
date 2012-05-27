#pragma once
#include <Eigen/Dense>
#include <simulation/basicobjects.h>
#include <simulation/softbodies.h>
#include <simulation/rope.h>
#include "sparse_utils.h"

class TrackedObject {
public:
  typedef boost::shared_ptr<TrackedObject> Ptr;
  EnvironmentObject::Ptr m_sim;
  Eigen::VectorXf m_sigs;
  int m_nNodes;
  std::string m_type;

  TrackedObject(EnvironmentObject::Ptr sim) : m_sim(sim) {}
  virtual Eigen::MatrixXf getPoints() = 0;
  virtual void applyEvidence(const SparseMatrixf& corr, const Eigen::MatrixXf& obsPts) = 0;
  virtual EnvironmentObject* getSim() {return m_sim.get();}
};

class TrackedRope : public TrackedObject { 
public:
  typedef boost::shared_ptr<TrackedRope> Ptr;

  TrackedRope(CapsuleRope::Ptr sim);

  Eigen::MatrixXf getPoints();
  void applyEvidence(const SparseMatrixf& corr, const Eigen::MatrixXf& obsPts);
  CapsuleRope* getSim() {return dynamic_cast<CapsuleRope*>(m_sim.get());}

protected:
  Eigen::VectorXf m_masses;
};


class TrackedTowel : public TrackedObject {
public:
  typedef boost::shared_ptr<TrackedTowel> Ptr;
  TrackedTowel(BulletSoftObject::Ptr, int xres, int yres);
  Eigen::MatrixXf getPoints();
  void applyEvidence(const SparseMatrixf& corr, const Eigen::MatrixXf& obsPts); // add forces
  BulletSoftObject* getSim() {return dynamic_cast<BulletSoftObject*>(m_sim.get());};

protected:
  std::vector<int> m_node2vert; // maps node index to bullet vertex index
  std::vector< std::vector<int> > m_vert2nodes; // maps bullet vertex index to indices of nodes
  Eigen::VectorXf m_masses;

};

std::vector<btVector3> calcImpulsesDamped(const std::vector<btVector3>& estPos, const std::vector<btVector3>& estVel, 
    const std::vector<btVector3>& obsPts, const SparseMatrixf& corr, const vector<float>& masses, float kp, float kd);
    

