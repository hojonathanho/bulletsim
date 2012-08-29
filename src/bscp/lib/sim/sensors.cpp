#include "sensors.h"

VectorXd CarBeaconFunc(const VectorXd& x) {
  return x.segment(0,2);
}

