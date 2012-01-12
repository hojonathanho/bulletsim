#include "basicobjects.h"
#include "softbodies.h"
#include "plotting.h"

vector<btVector3> clothOptImpulses(BulletSoftObject::Ptr cloth, const vector<btVector3>& pts);
void applyImpulses(const vector<btVector3>& impulses, BulletSoftObject::Ptr);

vector<btVector3> getSoftBodyNodes(BulletSoftObject::Ptr psb);


void initTrackingPlots();

struct plots {
  static PlotLines::Ptr linesAB;
  static PlotLines::Ptr linesBA;
};

