#ifndef __FL_FOLDING_H__
#define __FL_FOLDING_H__

#include "simulation/plotting.h"
#include "utils/config.h"

class Scene;
class Cloth;
class btVector3;

struct FoldingConfig : Config {
    static int foldHalfCircleDivs;
    static int attractNodesMaxSteps;
    static float attractNodesMaxAvgErr;
    static bool disableGravityWhenFolding;

    static float dropHeight;

    FoldingConfig() : Config() {
        params.push_back(new Parameter<int>("foldHalfCircleDivs", &foldHalfCircleDivs, "steps around half-circle when folding"));
        params.push_back(new Parameter<int>("attractNodesMaxSteps", &attractNodesMaxSteps, "max physics steps when attracting cloth to desired folding pos"));
        params.push_back(new Parameter<float>("attractNodesMaxAvgErr", &attractNodesMaxAvgErr, "max avg node pos error when attracting cloth to desired folding pos"));
        params.push_back(new Parameter<bool>("disableGravityWhenFolding", &disableGravityWhenFolding, "disable gravity when folding"));

        params.push_back(new Parameter<float>("dropHeight", &dropHeight, "height to pick up cloth and drop from"));
    }
};

namespace Folding {

// z-components of a and b are ignored
// a line on the cloth is drawn from point a to point b
// and the left side is folded over to the right side
void foldClothAlongLine(Scene &scene, Cloth &c, const btVector3 &a, const btVector3 &b);

// picks up the cloth to FoldingConfig::dropHeight,
// rotates the cloth randomly, and lets it drop
void pickUpAndDrop(Scene &scene, Cloth &cloth);

}

#endif // __FL_FOLDING_H__
