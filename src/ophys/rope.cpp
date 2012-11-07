#include "rope.h"

namespace ophys {

Rope::Rope(int npts, double length, double radius) : m_npts(npts), m_length(length), m_radius(radius), m_seglen(length/(npts-1.)) {
  for (int i = 0; i < m_npts - 1; ++i) {
    m_colobjs.push_back(new btCollisionObject());
  }
}

void Rope::addVars(GRBModel &grbm) const {
  boost::format fmt("rope_%s_%d_%c");
  assert(m_xvars.size() == 0);
  for (int i = 0; i < m_npts; ++i) {
    m_xvars.push_back(grbm.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (fmt % "myrope" % i % 'x').str()));
    m_xvars.push_back(grbm.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (fmt % "myrope" % i % 'y').str()));
    m_xvars.push_back(grbm.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (fmt % "myrope" % i % 'z').str()));
  }
}

void Rope::addQConstrs(GRBModel &grbm) const {
  // enforce segment rigidity
  boost::format fmt("rope_%s_seglen_%d_%c");
  for (int i = 0; i < m_npts - 1; ++i) {
    GRBQuadExpr dist2 =
      square(m_xvars[3*i    ] - m_xvars[3*(i+1)    ]) +
      square(m_xvars[3*i + 1] - m_xvars[3*(i+1) + 1]) +
      square(m_xvars[3*i + 2] - m_xvars[3*(i+1) + 2]);
    grbm.addQConstr(dist2 == m_seglen*m_seglen, (fmt % "myrope" % i).str());
  }
}

void Rope::getCostTerm() const {
  // 
}


} // namespace ophys
