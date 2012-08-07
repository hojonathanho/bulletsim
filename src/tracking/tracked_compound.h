#include "tracked_object.h"

typedef CompoundObject<BulletObject> GenericCompoundObject;

class TrackedArticulated : public TrackedObject {
public:
  typedef boost::shared_ptr<TrackedArticulated> Ptr;

  std::vector<btVector3> m_relativePos;
  std::vector<int> m_ownerInd;

  TrackedArticulated(GenericCompoundObject::Ptr sim);

  std::vector<btVector3> getPoints();
  const Eigen::VectorXf getPriorDist();
  void applyEvidence(const Eigen::MatrixXf& corr, const Eigen::MatrixXf& obsPts);
  CapsuleRope* getSim() {return dynamic_cast<CapsuleRope*>(m_sim.get());}
  cv::Mat makeTexture(ColorCloudPtr cloud);
  void initColors();

protected:
  Eigen::VectorXf m_masses;
};



