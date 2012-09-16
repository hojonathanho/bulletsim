#include "tracked_object.h"

typedef CompoundObject<BulletObject> GenericCompoundObject;

class TrackedCompound : public TrackedObject {
public:
  typedef boost::shared_ptr<TrackedCompound> Ptr;

  std::vector<btVector3> m_relativePos;
  int m_nBodies;
  std::map<int, std::vector<int> > m_body2nodes;
  std::vector<int> m_node2body;

  TrackedCompound(GenericCompoundObject::Ptr sim, btDynamicsWorld* world);

  std::vector<btVector3> getPoints();
  void applyEvidence(const Eigen::MatrixXf& corr, const Eigen::MatrixXf& obsPts);
  GenericCompoundObject* getSim() {return dynamic_cast<GenericCompoundObject*>(m_sim.get());}
  cv::Mat makeTexture(ColorCloudPtr cloud);
  void initColors();

protected:
  Eigen::VectorXf m_masses;
};



