#include "userconfig.h"

struct MyConfigData : public ConfigData {
  float atob;
  float btoa;
  float mult;
  float reg;
  float angDamping;
  float linDamping;
  int nIter;

  MyConfigData() {
    opts.add_options()
      OPT(atob, float, 1, "force multiplier from model to observation")
      OPT(btoa, float, 1, "force multiplier from observation to model")
      OPT(mult, float, 1, "overall force multiplier")
      OPT(reg, float, 1, "regularization factor to slow rope")
      OPT(angDamping, float, 1,"angular damping for rope")
      OPT(linDamping, float, .75,"linear damping for rope")
      OPT(nIter, int, 20, "num iterations")
      ;
  }

};

#define CFG2 (static_cast<MyConfigData*>(CFG))


