#include "tracked_object.h"

typedef CompoundObject<BulletObject> GenericCompoundObject;

class TrackedCompound : public TrackedObject {
public:
  typedef boost::shared_ptr<TrackedCompound> Ptr;

  std::vector<btVector3> m_relativePos;
  std::vector<int> m_ownerInd;

  TrackedCompound(GenericCompoundObject::Ptr sim);

  std::vector<btVector3> getPoints();
  const Eigen::VectorXf getPriorDist();
  void applyEvidence(const Eigen::MatrixXf& corr, const Eigen::MatrixXf& obsPts);
  CapsuleRope* getSim() {return dynamic_cast<CapsuleRope*>(m_sim.get());}
  cv::Mat makeTexture(ColorCloudPtr cloud);
  void initColors();

protected:
  Eigen::VectorXf m_masses;
};



