#pragma once

namespace ophys {

class CapsuleObject {
public:
  virtual VarPVec getVars();
  virtual ConstrVec getConstrs();
  virtual ConstrVec getQConstrs();

private:
};

} // namespace ophys
