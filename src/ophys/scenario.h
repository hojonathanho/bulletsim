#pragma once

#include "optrope_state.h"

class Scenario {
public:
  virtual ~Scenario() { }

  virtual const MatrixX3d &getInitialRopePoints() = 0;
  virtual double getRopeLinkLen() = 0;
  virtual const Vector7d &getInitManipDofs() = 0;
  virtual double goalCost(const OptRopeState &s) = 0;
  virtual bool disableRopeCnt() { return false; }
};
