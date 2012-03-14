#ifndef __CLOTHMANIP_H__
#define __CLOTHMANIP_H__

#include "simulation/environment.h"
#include "simulation/basicobjects.h"

class Scene;
class btSoftBody;

struct ClothSpec {
    btSoftBody *psb;
    int resx, resy;
};

// utility functions for manipulating cloth
void liftClothEdges(Scene &scene, ClothSpec &cs, bool disableDrawing=true);
void liftClothMiddle(Scene &scene, ClothSpec &cs, bool disableDrawing=true);
void randomizeCloth(Scene &scene, btSoftBody *psb, int resx, int resy, int numSteps=20);


#endif // __CLOTHMANIP_H__
