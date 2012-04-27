#ifndef __FL_FOLDING_H__
#define __FL_FOLDING_H__

#include "simulation/plotting.h"
class Scene;
class Cloth;
class btVector3;

// z-components of a and b are ignored
void foldClothAlongLine(Scene &scene, PlotPoints::Ptr destplot, Cloth &c, const btVector3 &a, const btVector3 &b);

#endif // __FL_FOLDING_H__
