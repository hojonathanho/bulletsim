#ifndef __OPTPHYSICS_ROPE_H__
#define __OPTPHYSICS_ROPE_H__

#include "ophys_common.h"

namespace ophys {

class Rope {
public:
  Rope(int npts, double length, double radius);
  ~Rope();

  static FromCapsuleRope(CapsuleRope::Ptr);

  virtual void addVars(GRBModel &) const;
  virtual ConstrVec getConstrs() const;
  virtual ConstrVec getQConstrs() const;

private:
  const int m_npts;
  const double m_radius;
  const double m_length;
  const double m_seglen;
  VarVec m_xvars;
  vector<boost::shared_ptr<btCollisionObject> > m_colobjs;
};


} // namespace ophys

#endif // __OPTPHYSICS_ROPE_H__
