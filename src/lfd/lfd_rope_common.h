#ifndef _LFD_ROPE_COMMON_H_
#define _LFD_ROPE_COMMON_H_

#include "rope_scenes.h"
#include "utils/logging.h"
#include "utils/config.h"
#include <vector>
using namespace std;

namespace lfd {

struct LFDRopeScene {
  LFDRopeScene(int argc, char *argv[], Config config);
  void resetScene(const vector<btVector3> &ropeCtlPts);

  boost::shared_ptr<TableRopeScene> scene;
};

typedef vector<btVector3> RopeState;
typedef map<string, vector<RopeState> > SegRopeStates; // map from seg_name to rope states at each timestep

// simulation world frame -> real pr2 base_footprint frame
RopeState ropeState_sim2real(const RopeState &, RaveRobotObject::Ptr pr2);
// real pr2 base_footprint frame -> simulation world frame
RopeState ropeState_real2sim(const RopeState &, RaveRobotObject::Ptr pr2);

struct RopeStatePlot : public PlotLines {
  typedef boost::shared_ptr<RopeStatePlot> Ptr;
  void setRope(const RopeState &rs, const Eigen::Vector3f &color, float alpha);
};

RopeState loadRopeStateFromDemoCloud(const string &demo_task, const string &demo_seg);

} // namespace lfd

#endif // _LFD_ROPE_COMMON_H_
