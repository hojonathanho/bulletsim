#include "simulation/softbodies.h"
#include "simulation/basicobjects.h"
#include <vector>


std::vector<btVector3> getNodes(BulletSoftObject::Ptr psb);
std::vector<float> getNodeMasses(BulletSoftObject::Ptr psb);
std::vector<btVector3> getNodeVels(BulletSoftObject::Ptr psb);
